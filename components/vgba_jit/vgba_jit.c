/* vgba_jit.c — M0: runtime-codegen self-test + exec arena + block cache.
 *
 * Infrastructure (RISC-V emitter, exec arena, block cache, I-cache flush) is
 * adapted from the hardware-validated mGBA dynarec in tab5-gba/gba_jit.c. It is
 * GBA-state-independent; later milestones (M1+) add the THUMB/ARM dispatch and
 * translation that read/write VBA-Next's global `bus.reg[]`. See DYNAREC_PLAN.md.
 */
#include "vgba_jit.h"
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "hal/cache_hal.h"
#include "esp_cache.h"

static const char *TAG = "VGBA_JIT";

/* Self-test arena holds ALL test blocks at UNIQUE addresses (no reuse — address
 * reuse leaves stale i-cache instrs on this P4 → intermittent illegal-instruction
 * crash). Live-dispatch arena is small (held during gameplay). Size is chosen per
 * vgba_jit_init() call. */
/* Single arena (32KB, fits exec-capable internal SRAM, stable to hold during
 * gameplay now that audio+cache are in PSRAM). The boot self-test uses the low
 * part at UNIQUE addresses; gameplay blocks APPEND after (no address reuse —
 * reuse is i-cache-unsafe on this P4). */
#define JIT_ARENA_BYTES (60 * 1024)  /* self-test holds every block at a unique addr (reuse is i-cache-unsafe on this P4) and uses ~40KB; 60KB fits the exec-capable internal SRAM (~64KB single-alloc cap; 96KB fails) and leaves a ~20KB tail where live JIT blocks APPEND (VGBA_JIT_ARM) without reusing self-test addresses */
#define JIT_CACHE_SIZE  1024   /* enough for the internal-SRAM arena's ~113 blocks */
/* A PSRAM exec arena was tried (2MB → 4000+ blocks) but PSRAM instruction-fetch is too
 * slow: once the working set exceeds the I-cache it thrashes → 28fps (< interpreter).
 * JIT blocks must live in FAST internal exec SRAM (capped ~64KB ≈ 113 blocks). */
#define VGBA_JIT_LDST   1    /* load/store in blocks. KNOWN residual risk (armed-mode only): a block-
                              * internal load/store runs its helper with cpuTotalTicks/reg[15] FROZEN at
                              * block entry (gba.c does lump cycle accounting only after the block returns).
                              * A LOAD from a timer counter reg returns 0xFFFF-((timerTicks-cpuTotalTicks)
                              * >>reload) (gba.c:1034-43) and open-bus reads use reg[15] → both would diverge
                              * with stale state. EMPIRICALLY this never bit (VGBA_JIT_VALIDATE = 0 mismatches
                              * over real gameplay — hot ROM loops don't load timer/open-bus regs). Kept =1
                              * because: (a) the JIT is only ARMED in research builds (shipped = disarmed, so
                              * no exposure), and (b) LDST=0 terminates blocks at every load/store → the hot
                              * loop fragments → armed JIT drops to interpreter speed (HW: peak 59 vs 88,
                              * roaming 27), negating the JIT. The proper fix BEFORE shipping the JIT armed =
                              * per-op state sync (write cpuTotalTicks+reg[15] before each ldst helper call). */
#define VGBA_JIT_BRANCH 1    /* translate B/Bcc as block terminators (bisect: 0 = branches→interp) */
#define VGBA_JIT_NEWOPS 1    /* bisect: 0 = drop this session's new ops (fmt-4 ADC/SBC/NEG/CMP/CMN/MUL + fmt-5 hi-reg) */
#define VGBA_JIT_NOPEFFECT 0 /* diag: blocks advance PC but apply NO data effects (exec-mechanism vs op-effects) */
#define JIT_CACHE_MASK  (JIT_CACHE_SIZE - 1)
#ifndef VGBA_PROFILE
#define VGBA_PROFILE    0    /* 1 = after self-test, arm measurement-only mode (g_vgba_jit=2):
                             * region + coverage profiler, interpreter still runs (no codegen).
                             * Build: idf.py -DVGBA_PROFILE=1 build (pair with VGBA_AUTOINPUT). */
#endif
#ifndef VGBA_JIT_ARM
#define VGBA_JIT_ARM    0    /* 1 = after self-test, ARM the chaining JIT (g_vgba_jit=1): real native
                             * block execution on ROM (M6). RESEARCH-ONLY: on koudai the JIT is arena-
                             * bound (internal exec-SRAM single-alloc ~64KB cap, can't grow; address
                             * reuse is i-cache-unsafe so can't recycle) → with the 60fps limiter it
                             * does not raise playable FPS. Default 0 = shipped interpreter. */
#endif

typedef struct { uint32_t pc; void *code; uint16_t ninstr; uint16_t flags; } jit_entry_t;

static uint8_t     *s_arena      = NULL;
static size_t       s_arena_used = 0;
static size_t       s_arena_size = 0;
static jit_entry_t  *s_cache     = NULL;

/* THUMB PC is halfword-aligned; index the cache by (pc>>1). */
static inline uint32_t jit_slot(uint32_t pc) { return (pc >> 1) & JIT_CACHE_MASK; }

bool vgba_jit_init(size_t arena_bytes, bool psram)
{
    if (s_arena && s_cache && s_arena_size >= arena_bytes) return true;
    if (s_arena) { heap_caps_free(s_arena); s_arena = NULL; }   /* re-size */
    if (psram) {
        /* PSRAM exec arena: MBs of room → cache the whole hot working set. The P4
         * instruction bus reaches PSRAM through the L2 cache; vgba_jit_sync_icache
         * (writeback D + invalidate I + fence.i) makes freshly-written blocks fetchable.
         * Try EXEC-capable PSRAM, else plain SPIRAM. */
        s_arena = heap_caps_aligned_alloc(64, arena_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_EXEC);
        if (!s_arena) s_arena = heap_caps_aligned_alloc(64, arena_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    } else {
        /* 64-byte aligned base so every block (also 64B-rounded in vgba_jit_alloc)
         * starts on its own I-cache line — no two blocks share a line. */
        s_arena = heap_caps_aligned_alloc(64, arena_bytes, MALLOC_CAP_EXEC);  /* internal exec SRAM */
    }
    if (!s_cache)  /* cache → INTERNAL SRAM: the per-instruction lookup is hot;
                    * a PSRAM cache was the 111ms killer (70K lookups/frame). */
        s_cache = heap_caps_calloc(JIT_CACHE_SIZE, sizeof(jit_entry_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_arena || !s_cache) {
        ESP_LOGE(TAG, "JIT init failed (arena=%p cache=%p)", (void *)s_arena, (void *)s_cache);
        return false;
    }
    s_arena_size = arena_bytes;
    s_arena_used = 0;
    ESP_LOGI(TAG, "JIT init: %uKB %s arena @%p, %d-entry cache",
             (unsigned)(arena_bytes / 1024), psram ? "PSRAM-exec" : "internal-exec",
             (void *)s_arena, JIT_CACHE_SIZE);
    return true;
}

void vgba_jit_release(void)
{
    if (s_arena) { heap_caps_free(s_arena); s_arena = NULL; }
    if (s_cache) { heap_caps_free(s_cache); s_cache = NULL; }
    s_arena_used = 0; s_arena_size = 0;
}

void vgba_jit_reset(void)
{
    s_arena_used = 0;
    if (s_cache) memset(s_cache, 0, JIT_CACHE_SIZE * sizeof(jit_entry_t));
}

void *vgba_jit_alloc(size_t bytes)
{
    /* Round each block up to a full 64-byte I-cache line. Since the arena base is
     * 64-byte aligned (vgba_jit_init) and s_arena_used stays a multiple of 64,
     * every block starts on its own cache line and no two blocks ever share one.
     * Sharing is unsafe on this P4: writing+invalidating a freshly-built neighbor
     * evicts the line under an already-executing chained block → intermittent
     * illegal-instruction crash. */
    const size_t LINE = 64;
    size_t aligned = (bytes + (LINE - 1)) & ~(LINE - 1);
    if (!s_arena || s_arena_used + aligned > s_arena_size) return NULL;
    void *p = s_arena + s_arena_used;
    s_arena_used += aligned;
    return p;
}

void vgba_jit_register(uint32_t pc, void *code)
{
    if (!s_cache) return;
    jit_entry_t *e = &s_cache[jit_slot(pc)];
    e->pc = pc;
    e->code = code;
}

void *vgba_jit_lookup(uint32_t pc)
{
    if (!s_cache) return NULL;
    jit_entry_t *e = &s_cache[jit_slot(pc)];
    return (e->code && e->pc == pc) ? e->code : NULL;
}

/* Make freshly-written code executable on ESP32-P4: write back L1 D-cache,
 * invalidate L1 I-cache over whole 64B lines, then fence.i to flush this hart's
 * instruction stream. (Proven recipe from the mGBA dynarec.) */
void vgba_jit_sync_icache(void *buf, size_t len)
{
    const uint32_t LINE = 64;
    uint32_t start = (uint32_t)buf & ~(LINE - 1);
    uint32_t end   = ((uint32_t)buf + len + LINE - 1) & ~(LINE - 1);
    /* SYNCHRONOUS writeback of the generated code to memory. cache_hal_writeback
     * alone wasn't enough for a PSRAM arena (the writeback didn't land before the
     * i-fetch → stale → instruction-access fault at a PSRAM addr). esp_cache_msync
     * (C2M) blocks until the data is in memory, for both internal SRAM and PSRAM. */
    esp_cache_msync(buf, len, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    cache_hal_invalidate_addr(start, end - start);
    __asm__ volatile("fence rw, rw\n\tfence.i" ::: "memory");
}

/* ── CPU state addresses, registered by gba.c (M2) ──
 * The emitter bakes these absolute addresses into generated code. */
static uint32_t *s_reg = NULL;            /* &bus.reg[0].I (GPR base, stride 4) */
static uint8_t  *s_nf = NULL, *s_cf = NULL, *s_zf = NULL, *s_vf = NULL; /* N/C/Z/V_FLAG */
static uint8_t  *s_rom = NULL;            /* ROM buffer (for opcode decode)     */
static uint32_t  s_rom_mask = 0;          /* (pc & s_rom_mask)&~1 = ROM offset  */
static uint8_t  *s_iwram = NULL;          /* internalRAM (0x03, 32KB)           */
static uint8_t  *s_ewram = NULL;          /* workRAM (0x02, 256KB)              */

void vgba_jit_set_state(uint32_t *reg, uint8_t *n, uint8_t *c, uint8_t *z, uint8_t *v,
                        uint8_t *rom, uint32_t rom_mask, uint8_t *iwram, uint8_t *ewram)
{
    s_reg = reg; s_nf = n; s_cf = c; s_zf = z; s_vf = v;
    s_rom = rom; s_rom_mask = rom_mask; s_iwram = iwram; s_ewram = ewram;
}

/* Memory-access helpers (registered by gba.c); the JIT emits CALLs to these. */
static void *s_h_ld32, *s_h_ld16, *s_h_ld16s, *s_h_ld8, *s_h_ld8s, *s_h_st32, *s_h_st16, *s_h_st8;
void vgba_jit_set_helpers(void *ld32, void *ld16, void *ld16s, void *ld8, void *ld8s,
                          void *st32, void *st16, void *st8)
{
    s_h_ld32 = ld32; s_h_ld16 = ld16; s_h_ld16s = ld16s; s_h_ld8 = ld8; s_h_ld8s = ld8s;
    s_h_st32 = st32; s_h_st16 = st16; s_h_st8 = st8;
}

/* Fetch a THUMB halfword opcode at `pc` from the right region buffer. */
static inline uint16_t jit_fetch(uint32_t pc)
{
    uint32_t r = pc >> 24;
    if (r == 0x03) return *(uint16_t *)(s_iwram + ((pc & 0x7FFFu)  & ~1u));   /* IWRAM 32KB  */
    if (r == 0x02) return *(uint16_t *)(s_ewram + ((pc & 0x3FFFFu) & ~1u));   /* EWRAM 256KB */
    return *(uint16_t *)(s_rom + ((pc & s_rom_mask) & ~1u));                   /* ROM 0x08+   */
}

/* 32-bit ARM opcode fetch (same regions, 4-aligned). */
static inline uint32_t jit_fetch32(uint32_t pc)
{
    uint32_t r = pc >> 24;
    if (r == 0x03) return *(uint32_t *)(s_iwram + ((pc & 0x7FFFu)  & ~3u));
    if (r == 0x02) return *(uint32_t *)(s_ewram + ((pc & 0x3FFFFu) & ~3u));
    return *(uint32_t *)(s_rom + ((pc & s_rom_mask) & ~3u));
}

/* ── Dispatch state (the dispatch function is defined after the emitters). ── */
int g_vgba_jit = 0;
static int s_arena_full = 0;         /* set once the live arena fills → stop build attempts */
unsigned g_vgba_blocks = 0;          /* total JIT blocks built (stats) */
unsigned g_vgba_arm_instrs = 0;      /* M-A1: ARM instrs run via JIT (coverage stat) */
unsigned g_arm_region[16] = {0};     /* M-A2 diag: ARM instrs executed per address region (>>24 & 0xF) */
unsigned g_vgba_jit_instrs = 0;      /* total THUMB instrs run via JIT (stats) */

/* ── Minimal RV32 encoder (adapted from the mGBA dynarec) ── */
enum { RV_ZERO = 0, RV_RA = 1, RV_T0 = 5, RV_T1 = 6, RV_T2 = 7 };
static inline uint32_t rv_lui (int rd, uint32_t up20)     { return (up20 << 12) | (rd << 7) | 0x37; }
static inline uint32_t rv_addi(int rd, int rs1, int imm)  { return ((imm & 0xFFF) << 20) | (rs1 << 15) | (0 << 12) | (rd << 7) | 0x13; }
static inline uint32_t rv_sw  (int rs2, int rs1, int imm) { int i = imm & 0xFFF; return (((i >> 5) & 0x7F) << 25) | (rs2 << 20) | (rs1 << 15) | (2 << 12) | ((i & 0x1F) << 7) | 0x23; }
static inline uint32_t rv_sb  (int rs2, int rs1, int imm) { int i = imm & 0xFFF; return (((i >> 5) & 0x7F) << 25) | (rs2 << 20) | (rs1 << 15) | (0 << 12) | ((i & 0x1F) << 7) | 0x23; }
static inline uint32_t rv_jalr(int rd, int rs1, int imm)  { return ((imm & 0xFFF) << 20) | (rs1 << 15) | (0 << 12) | (rd << 7) | 0x67; }
static inline uint32_t rv_lw   (int rd, int rs1, int imm) { return ((imm & 0xFFF) << 20) | (rs1 << 15) | (2 << 12) | (rd << 7) | 0x03; }
static inline uint32_t rv_add  (int rd, int rs1, int rs2) { return (rs2 << 20) | (rs1 << 15) | (0 << 12) | (rd << 7) | 0x33; }
static inline uint32_t rv_sub  (int rd, int rs1, int rs2) { return (0x20u << 25) | (rs2 << 20) | (rs1 << 15) | (0 << 12) | (rd << 7) | 0x33; }
static inline uint32_t rv_xor  (int rd, int rs1, int rs2) { return (rs2 << 20) | (rs1 << 15) | (4 << 12) | (rd << 7) | 0x33; }
static inline uint32_t rv_and  (int rd, int rs1, int rs2) { return (rs2 << 20) | (rs1 << 15) | (7 << 12) | (rd << 7) | 0x33; }
static inline uint32_t rv_sltu (int rd, int rs1, int rs2) { return (rs2 << 20) | (rs1 << 15) | (3 << 12) | (rd << 7) | 0x33; }
static inline uint32_t rv_xori (int rd, int rs1, int imm) { return ((imm & 0xFFF) << 20) | (rs1 << 15) | (4 << 12) | (rd << 7) | 0x13; }
static inline uint32_t rv_sltiu(int rd, int rs1, int imm) { return ((imm & 0xFFF) << 20) | (rs1 << 15) | (3 << 12) | (rd << 7) | 0x13; }
static inline uint32_t rv_srli (int rd, int rs1, int sh)  { return ((sh & 0x1F) << 20) | (rs1 << 15) | (5 << 12) | (rd << 7) | 0x13; }
static inline uint32_t rv_slli (int rd, int rs1, int sh)  { return ((sh & 0x1F) << 20) | (rs1 << 15) | (1 << 12) | (rd << 7) | 0x13; }
static inline uint32_t rv_srai (int rd, int rs1, int sh)  { return ((0x400u | (sh & 0x1F)) << 20) | (rs1 << 15) | (5 << 12) | (rd << 7) | 0x13; }
static inline uint32_t rv_or   (int rd, int rs1, int rs2) { return (rs2 << 20) | (rs1 << 15) | (6 << 12) | (rd << 7) | 0x33; }
static inline uint32_t rv_andi (int rd, int rs1, int imm) { return ((imm & 0xFFF) << 20) | (rs1 << 15) | (7 << 12) | (rd << 7) | 0x13; }
static inline uint32_t rv_lbu  (int rd, int rs1, int imm) { return ((imm & 0xFFF) << 20) | (rs1 << 15) | (4 << 12) | (rd << 7) | 0x03; }
static inline uint32_t rv_mul  (int rd, int rs1, int rs2) { return (0x01u << 25) | (rs2 << 20) | (rs1 << 15) | (0 << 12) | (rd << 7) | 0x33; } /* RV32M */
static inline uint32_t rv_beq  (int rs1, int rs2, int imm) { uint32_t i = (uint32_t)imm;     /* B-type, byte offset (even) */
    return (((i>>12)&1)<<31) | (((i>>5)&0x3F)<<25) | (rs2<<20) | (rs1<<15) | (0<<12) | (((i>>1)&0xF)<<8) | (((i>>11)&1)<<7) | 0x63; }
enum { RV_T3 = 28, RV_T4 = 29, RV_T5 = 30, RV_A0 = 10, RV_A1 = 11 };

/* Load a 32-bit constant (lui+addi, lo12 sign-extension corrected). */
static int emit_li(uint32_t *c, int n, int rd, uint32_t val)
{
    uint32_t hi = (val + 0x800u) >> 12;
    int32_t  lo = (int32_t)val - (int32_t)(hi << 12);
    c[n++] = rv_lui(rd, hi & 0xFFFFF);
    c[n++] = rv_addi(rd, rd, lo);
    return n;
}

/* store a 0/1 flag byte from register `src` to absolute address `addr` (clobbers t1). */
static int emit_store_flag(uint32_t *c, int n, uint8_t *addr, int src)
{
    n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)addr);
    c[n++] = rv_sb(src, RV_T1, 0);
    return n;
}

/* Emit a self-contained leaf block for a THUMB format-3 immediate ALU op on Rd:
 *   op 0 MOV: Rd=imm        ; N=imm>>31, Z=(imm==0)            ; C,V unchanged
 *   op 1 CMP: res=Rd-imm    ; N,Z,C,V set (no reg write)
 *   op 2 ADD: Rd=Rd+imm     ; N,Z,C,V set
 *   op 3 SUB: Rd=Rd-imm     ; N,Z,C,V set
 * Flag math matches VBA-Next's NEG/POS formulas exactly via standard RISC-V
 * (C_add=res<lhs ; C_sub=!(lhs<rhs) ; V=((a^res)&(b^res))>>31 with b=imm for add,
 * b=rhs for sub form). Addresses baked in; called as void fn(void). */
static int emit_alu_imm(uint32_t *c, int op, int rd, uint32_t imm, uint32_t *reg,
                        uint8_t *nf, uint8_t *cf, uint8_t *zf, uint8_t *vf)
{
    int n = 0;
    if (op == 0) {                                          /* MOV */
        n = emit_li(c, n, RV_T0, imm);
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rd));
        c[n++] = rv_sw(RV_T0, RV_T1, 0);                   /* reg[Rd] = imm */
        c[n++] = rv_addi(RV_T4, RV_ZERO, 0);
        n = emit_store_flag(c, n, nf, RV_T4);              /* N = 0 (imm8) */
        c[n++] = rv_sltiu(RV_T4, RV_T0, 1);
        n = emit_store_flag(c, n, zf, RV_T4);              /* Z = (imm==0) */
        c[n++] = rv_jalr(RV_ZERO, RV_RA, 0);
        return n;
    }
    n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rd));
    c[n++] = rv_lw(RV_T0, RV_T1, 0);                       /* t0 = lhs = reg[Rd] */
    n = emit_li(c, n, RV_T2, imm);                         /* t2 = imm           */
    if (op == 2) {                                         /* ADD */
        c[n++] = rv_add(RV_T3, RV_T0, RV_T2);             /* res */
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rd));
        c[n++] = rv_sw(RV_T3, RV_T1, 0);                  /* reg[Rd] = res */
        c[n++] = rv_sltu(RV_T4, RV_T3, RV_T0);            /* C = res<lhs */
        n = emit_store_flag(c, n, cf, RV_T4);
        c[n++] = rv_xor(RV_T4, RV_T0, RV_T3);             /* V = ((lhs^res)&(imm^res))>>31 */
        c[n++] = rv_xor(RV_T5, RV_T2, RV_T3);
        c[n++] = rv_and(RV_T4, RV_T4, RV_T5);
        c[n++] = rv_srli(RV_T4, RV_T4, 31);
        n = emit_store_flag(c, n, vf, RV_T4);
    } else {                                              /* SUB(3) / CMP(1) */
        c[n++] = rv_sub(RV_T3, RV_T0, RV_T2);             /* res = lhs - imm */
        if (op == 3) {
            n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rd));
            c[n++] = rv_sw(RV_T3, RV_T1, 0);              /* reg[Rd] = res */
        }
        c[n++] = rv_sltu(RV_T4, RV_T0, RV_T2);            /* C = !(lhs<imm) */
        c[n++] = rv_xori(RV_T4, RV_T4, 1);
        n = emit_store_flag(c, n, cf, RV_T4);
        c[n++] = rv_xor(RV_T4, RV_T0, RV_T2);             /* V = ((lhs^imm)&(lhs^res))>>31 */
        c[n++] = rv_xor(RV_T5, RV_T0, RV_T3);
        c[n++] = rv_and(RV_T4, RV_T4, RV_T5);
        c[n++] = rv_srli(RV_T4, RV_T4, 31);
        n = emit_store_flag(c, n, vf, RV_T4);
    }
    c[n++] = rv_srli(RV_T4, RV_T3, 31);                   /* N = res>>31 */
    n = emit_store_flag(c, n, nf, RV_T4);
    c[n++] = rv_sltiu(RV_T4, RV_T3, 1);                   /* Z = (res==0) */
    n = emit_store_flag(c, n, zf, RV_T4);
    c[n++] = rv_jalr(RV_ZERO, RV_RA, 0);
    return n;
}

/* Emit a leaf block for THUMB format-2 `ADD/SUB Rd, Rs, <Rn|#imm3>`:
 *   Rd = Rs (+|-) rhs ; full N/Z/C/V (same flag math as emit_alu_imm).
 * rhs_is_imm: 1 → rhs is an immediate; 0 → rhs is reg index Rn. */
static int emit_addsub(uint32_t *c, int is_sub, int rd, int rs, int rhs_is_imm, uint32_t rhs,
                       uint32_t *reg, uint8_t *nf, uint8_t *cf, uint8_t *zf, uint8_t *vf)
{
    int n = 0;
    n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rs));
    c[n++] = rv_lw(RV_T0, RV_T1, 0);                      /* t0 = lhs = reg[Rs] */
    if (rhs_is_imm) {
        n = emit_li(c, n, RV_T2, rhs);                   /* t2 = imm */
    } else {
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rhs));
        c[n++] = rv_lw(RV_T2, RV_T1, 0);                 /* t2 = reg[Rn] */
    }
    c[n++] = is_sub ? rv_sub(RV_T3, RV_T0, RV_T2) : rv_add(RV_T3, RV_T0, RV_T2);  /* res */
    n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rd));
    c[n++] = rv_sw(RV_T3, RV_T1, 0);                     /* reg[Rd] = res */
    if (!is_sub) {
        c[n++] = rv_sltu(RV_T4, RV_T3, RV_T0);           /* C = res<lhs */
        n = emit_store_flag(c, n, cf, RV_T4);
        c[n++] = rv_xor(RV_T4, RV_T0, RV_T3);            /* V */
        c[n++] = rv_xor(RV_T5, RV_T2, RV_T3);
        c[n++] = rv_and(RV_T4, RV_T4, RV_T5);
        c[n++] = rv_srli(RV_T4, RV_T4, 31);
        n = emit_store_flag(c, n, vf, RV_T4);
    } else {
        c[n++] = rv_sltu(RV_T4, RV_T0, RV_T2);           /* C = !(lhs<rhs) */
        c[n++] = rv_xori(RV_T4, RV_T4, 1);
        n = emit_store_flag(c, n, cf, RV_T4);
        c[n++] = rv_xor(RV_T4, RV_T0, RV_T2);            /* V */
        c[n++] = rv_xor(RV_T5, RV_T0, RV_T3);
        c[n++] = rv_and(RV_T4, RV_T4, RV_T5);
        c[n++] = rv_srli(RV_T4, RV_T4, 31);
        n = emit_store_flag(c, n, vf, RV_T4);
    }
    c[n++] = rv_srli(RV_T4, RV_T3, 31);                  /* N */
    n = emit_store_flag(c, n, nf, RV_T4);
    c[n++] = rv_sltiu(RV_T4, RV_T3, 1);                  /* Z */
    n = emit_store_flag(c, n, zf, RV_T4);
    c[n++] = rv_jalr(RV_ZERO, RV_RA, 0);
    return n;
}

/* Emit a leaf block for THUMB format-4 logical op (Rd = Rd op Rs), N/Z only,
 * C/V unchanged. op: 0 AND, 1 EOR, 2 ORR, 3 BIC, 4 MVN, 5 TST(no write). */
static int emit_logic(uint32_t *c, int op, int rd, int rs, uint32_t *reg, uint8_t *nf, uint8_t *zf)
{
    int n = 0;
    n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rs));
    c[n++] = rv_lw(RV_T2, RV_T1, 0);                       /* t2 = reg[Rs] */
    if (op != 4) {                                         /* MVN ignores Rd input */
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rd));
        c[n++] = rv_lw(RV_T0, RV_T1, 0);                  /* t0 = reg[Rd] */
    }
    switch (op) {
        case 0: case 5: c[n++] = rv_and(RV_T3, RV_T0, RV_T2); break;            /* AND / TST */
        case 1:         c[n++] = rv_xor(RV_T3, RV_T0, RV_T2); break;            /* EOR */
        case 2:         c[n++] = rv_or (RV_T3, RV_T0, RV_T2); break;            /* ORR */
        case 3:         c[n++] = rv_xori(RV_T5, RV_T2, -1); c[n++] = rv_and(RV_T3, RV_T0, RV_T5); break; /* BIC: Rd & ~Rs */
        case 4:         c[n++] = rv_xori(RV_T3, RV_T2, -1); break;              /* MVN: ~Rs */
    }
    if (op != 5) {                                        /* TST writes no reg */
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rd));
        c[n++] = rv_sw(RV_T3, RV_T1, 0);
    }
    c[n++] = rv_srli(RV_T4, RV_T3, 31);  n = emit_store_flag(c, n, nf, RV_T4);  /* N */
    c[n++] = rv_sltiu(RV_T4, RV_T3, 1);  n = emit_store_flag(c, n, zf, RV_T4);  /* Z */
    c[n++] = rv_jalr(RV_ZERO, RV_RA, 0);
    return n;
}

/* Emit a leaf block for THUMB format-1 shift `Rd = Rs <shift> #imm5` (shift is
 * compile-time, so no runtime branch). op: 0 LSL, 1 LSR, 2 ASR. Sets N/Z/C
 * (V unchanged); LSL#0 preserves C; LSR/ASR #0 mean shift-by-32. Matches VBA's
 * VALUE_*_IMM_C macros (C_OUT initialized to C, only overwritten when shifted). */
static int emit_shift_imm(uint32_t *c, int op, int rd, int rs, int shift,
                          uint32_t *reg, uint8_t *nf, uint8_t *cf, uint8_t *zf)
{
    int n = 0, set_c = 1;
    n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rs));
    c[n++] = rv_lw(RV_T0, RV_T1, 0);                       /* t0 = Rm */
    if (op == 0) {                                        /* LSL */
        if (shift == 0) { c[n++] = rv_addi(RV_T3, RV_T0, 0); set_c = 0; }   /* res=Rm, C kept */
        else { c[n++] = rv_srli(RV_T4, RV_T0, 32 - shift); c[n++] = rv_andi(RV_T4, RV_T4, 1);
               c[n++] = rv_slli(RV_T3, RV_T0, shift); }
    } else if (op == 1) {                                /* LSR */
        if (shift == 0) { c[n++] = rv_srli(RV_T4, RV_T0, 31); c[n++] = rv_addi(RV_T3, RV_ZERO, 0); } /* #32 */
        else { c[n++] = rv_srli(RV_T4, RV_T0, shift - 1); c[n++] = rv_andi(RV_T4, RV_T4, 1);
               c[n++] = rv_srli(RV_T3, RV_T0, shift); }
    } else {                                             /* ASR */
        if (shift == 0) { c[n++] = rv_srli(RV_T4, RV_T0, 31); c[n++] = rv_srai(RV_T3, RV_T0, 31); } /* #32 */
        else { c[n++] = rv_srai(RV_T4, RV_T0, shift - 1); c[n++] = rv_andi(RV_T4, RV_T4, 1);
               c[n++] = rv_srai(RV_T3, RV_T0, shift); }
    }
    n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rd));
    c[n++] = rv_sw(RV_T3, RV_T1, 0);                      /* reg[Rd] = res */
    if (set_c) n = emit_store_flag(c, n, cf, RV_T4);     /* C (not for LSL#0) */
    c[n++] = rv_srli(RV_T4, RV_T3, 31);  n = emit_store_flag(c, n, nf, RV_T4);  /* N */
    c[n++] = rv_sltiu(RV_T4, RV_T3, 1);  n = emit_store_flag(c, n, zf, RV_T4);  /* Z */
    c[n++] = rv_jalr(RV_ZERO, RV_RA, 0);
    return n;
}

/* Emit VBA-Next's EXACT add/sub carry+overflow into cf/vf from registers holding
 * a, b, res. is_sub selects SUB{CARRY,OVERFLOW} vs ADD{CARRY,OVERFLOW} (the
 * 3-term NEG/POS formulas in gba.c, not the sltu shortcut — required because ADC
 * /SBC carry-in makes `res<lhs` wrong). Clobbers t1,t4,t5,a0,a1; preserves the
 * ra/rb/rres inputs (callers keep a,b,res in t0,t2,t3). */
static int emit_addsub_flags(uint32_t *c, int n, int is_sub, int ra, int rb, int rres,
                             uint8_t *cf, uint8_t *vf)
{
    /* ── C ──  add: (na&nb)|pc&(na|nb)   sub: (na&pb)|pc&(na|pb)   (pc=POS(res)) */
    c[n++] = rv_srli(RV_T4, ra, 31);                       /* t4 = na = NEG(a) */
    c[n++] = rv_srli(RV_T5, rb, 31);                       /* t5 = NEG(b) */
    if (is_sub) c[n++] = rv_xori(RV_T5, RV_T5, 1);         /*   → pb = POS(b) */
    c[n++] = rv_srli(RV_A0, rres, 31);                     /* a0 = NEG(res) */
    c[n++] = rv_xori(RV_A0, RV_A0, 1);                     /*   → pc = POS(res) */
    c[n++] = rv_and(RV_T1, RV_T4, RV_T5);                  /* na & B */
    c[n++] = rv_or (RV_A1, RV_T4, RV_T5);                  /* na | B */
    c[n++] = rv_and(RV_A1, RV_A1, RV_A0);                  /* pc & (na|B) */
    c[n++] = rv_or (RV_A0, RV_T1, RV_A1);                  /* C → a0 (emit_store_flag clobbers t1!) */
    n = emit_store_flag(c, n, cf, RV_A0);
    /* ── V ──  add: (na&nb&pc)|(pa&pb&nc)   sub: (na&pb&pc)|(pa&nb&nc) */
    c[n++] = rv_srli(RV_T4, ra, 31);                       /* na */
    c[n++] = rv_srli(RV_T5, rb, 31);                       /* nb (sub: keep nb) */
    c[n++] = rv_srli(RV_A0, rres, 31);                     /* nc */
    if (!is_sub) {
        c[n++] = rv_xori(RV_A1, RV_A0, 1);                 /* pc */
        c[n++] = rv_and(RV_T1, RV_T4, RV_T5);              /* na&nb */
        c[n++] = rv_and(RV_T1, RV_T1, RV_A1);              /* term1 = na&nb&pc */
        c[n++] = rv_xori(RV_T4, RV_T4, 1);                 /* pa */
        c[n++] = rv_xori(RV_T5, RV_T5, 1);                 /* pb */
        c[n++] = rv_and(RV_A1, RV_T4, RV_T5);              /* pa&pb */
        c[n++] = rv_and(RV_A1, RV_A1, RV_A0);              /* term2 = pa&pb&nc */
    } else {
        c[n++] = rv_xori(RV_A1, RV_T5, 1);                 /* pb */
        c[n++] = rv_and(RV_T1, RV_T4, RV_A1);              /* na&pb */
        c[n++] = rv_xori(RV_A1, RV_A0, 1);                 /* pc */
        c[n++] = rv_and(RV_T1, RV_T1, RV_A1);              /* term1 = na&pb&pc */
        c[n++] = rv_xori(RV_T4, RV_T4, 1);                 /* pa */
        c[n++] = rv_and(RV_A1, RV_T4, RV_T5);              /* pa&nb */
        c[n++] = rv_and(RV_A1, RV_A1, RV_A0);              /* term2 = pa&nb&nc */
    }
    c[n++] = rv_or(RV_A0, RV_T1, RV_A1);                   /* V → a0 (emit_store_flag clobbers t1!) */
    n = emit_store_flag(c, n, vf, RV_A0);
    return n;
}

/* Emit a leaf block for THUMB format-4 ADC/SBC/NEG/CMN/CMP(reg) — full N/Z/C/V
 * with VBA's exact carry/overflow. op: 0 ADC, 1 SBC, 2 NEG, 3 CMN, 4 CMP.
 *   ADC: Rd = Rd + Rs + C   SBC: Rd = Rd - Rs - !C   NEG: Rd = 0 - Rs
 *   CMN: Rd + Rs (no write) CMP: Rd - Rs (no write). a,b,res live in t0,t2,t3. */
static int emit_alu_carry(uint32_t *c, int op, int rd, int rs, uint32_t *reg,
                          uint8_t *nf, uint8_t *cf, uint8_t *zf, uint8_t *vf)
{
    int n = 0;
    int is_sub = (op == 1 || op == 2 || op == 4);   /* SBC/NEG/CMP are subtractive */
    int writes = (op != 3 && op != 4);              /* CMN/CMP set flags only */
    if (op == 2) {                                  /* NEG: a=0, b=reg[Rs] */
        c[n++] = rv_addi(RV_T0, RV_ZERO, 0);
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rs));
        c[n++] = rv_lw(RV_T2, RV_T1, 0);
    } else {                                        /* a=reg[Rd], b=reg[Rs] */
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rd));
        c[n++] = rv_lw(RV_T0, RV_T1, 0);
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rs));
        c[n++] = rv_lw(RV_T2, RV_T1, 0);
    }
    if (op == 0) {                                  /* ADC: a + b + C */
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)cf);
        c[n++] = rv_lbu(RV_A0, RV_T1, 0);
        c[n++] = rv_add(RV_T3, RV_T0, RV_T2);
        c[n++] = rv_add(RV_T3, RV_T3, RV_A0);
    } else if (op == 1) {                           /* SBC: a - b - 1 + C */
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)cf);
        c[n++] = rv_lbu(RV_A0, RV_T1, 0);
        c[n++] = rv_sub(RV_T3, RV_T0, RV_T2);
        c[n++] = rv_addi(RV_T3, RV_T3, -1);
        c[n++] = rv_add(RV_T3, RV_T3, RV_A0);
    } else if (is_sub) {                            /* NEG / CMP: a - b */
        c[n++] = rv_sub(RV_T3, RV_T0, RV_T2);
    } else {                                        /* CMN: a + b */
        c[n++] = rv_add(RV_T3, RV_T0, RV_T2);
    }
    if (writes) {
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rd));
        c[n++] = rv_sw(RV_T3, RV_T1, 0);
    }
    n = emit_addsub_flags(c, n, is_sub, RV_T0, RV_T2, RV_T3, cf, vf);
    c[n++] = rv_srli(RV_T4, RV_T3, 31);  n = emit_store_flag(c, n, nf, RV_T4);  /* N */
    c[n++] = rv_sltiu(RV_T4, RV_T3, 1);  n = emit_store_flag(c, n, zf, RV_T4);  /* Z */
    c[n++] = rv_jalr(RV_ZERO, RV_RA, 0);
    return n;
}

/* Emit a leaf block for THUMB format-4 MUL: Rd = Rs * Rd; N/Z only, C/V kept. */
static int emit_mul(uint32_t *c, int rd, int rs, uint32_t *reg, uint8_t *nf, uint8_t *zf)
{
    int n = 0;
    n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rd));
    c[n++] = rv_lw(RV_T0, RV_T1, 0);                       /* t0 = reg[Rd] */
    n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rs));
    c[n++] = rv_lw(RV_T2, RV_T1, 0);                       /* t2 = reg[Rs] */
    c[n++] = rv_mul(RV_T3, RV_T0, RV_T2);                  /* res = Rs * Rd */
    n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rd));
    c[n++] = rv_sw(RV_T3, RV_T1, 0);
    c[n++] = rv_srli(RV_T4, RV_T3, 31);  n = emit_store_flag(c, n, nf, RV_T4);  /* N */
    c[n++] = rv_sltiu(RV_T4, RV_T3, 1);  n = emit_store_flag(c, n, zf, RV_T4);  /* Z */
    c[n++] = rv_jalr(RV_ZERO, RV_RA, 0);
    return n;
}

/* Emit a leaf block for THUMB format-5 hi-reg MOV/ADD (Rd = Rm, or Rd += Rm),
 * NO flags. Only the both-operands-≤r14 case reaches here — a PC dest/operand
 * (r15) or BX terminates the block in the caller (reg[15] is only valid at block
 * boundaries, never mid-block). is_add: 1 ADD, 0 MOV. */
static int emit_hireg_move(uint32_t *c, int is_add, int rd, int rm, uint32_t *reg)
{
    int n = 0;
    n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rm));
    c[n++] = rv_lw(RV_T0, RV_T1, 0);                       /* t0 = reg[Rm] */
    if (is_add) {
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rd));
        c[n++] = rv_lw(RV_T2, RV_T1, 0);                  /* t2 = reg[Rd] */
        c[n++] = rv_add(RV_T0, RV_T2, RV_T0);             /* t0 = Rd + Rm */
    }
    n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rd));
    c[n++] = rv_sw(RV_T0, RV_T1, 0);                      /* reg[Rd] = t0 */
    c[n++] = rv_jalr(RV_ZERO, RV_RA, 0);
    return n;
}

/* Emit a load/store body (no ret). `addr_const`>=0 → absolute address baked in
 * (format-6 PC-relative); else addr = reg[rb] + off. is_store/size(0=byte,1=half,
 * 2=word)/signed select the helper. reg[rd] is loaded(store)/stored(load). The
 * block must be NON-LEAF (prologue saves ra) since this emits a CALL. a0=addr/
 * result, a1=value. Returns new n, or -1 if a needed helper isn't registered. */
static int emit_ldst(uint32_t *c, int n, int is_store, int size, int is_signed,
                     int rd, int rb, int off, long addr_const)
{
    void *h = is_store ? (size == 0 ? s_h_st8 : size == 1 ? s_h_st16 : s_h_st32)
                       : (size == 2 ? s_h_ld32
                          : size == 1 ? (is_signed ? s_h_ld16s : s_h_ld16)
                                      : (is_signed ? s_h_ld8s : s_h_ld8));
    if (!h) return -1;
    /* a0 = address */
    if (addr_const >= 0) {
        n = emit_li(c, n, RV_A0, (uint32_t)addr_const);
    } else {
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(s_reg + rb));
        c[n++] = rv_lw(RV_A0, RV_T1, 0);
        if (off) c[n++] = rv_addi(RV_A0, RV_A0, off);
    }
    if (is_store) {
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(s_reg + rd));
        c[n++] = rv_lw(RV_A1, RV_T1, 0);                  /* a1 = reg[Rd] */
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)h);
        c[n++] = rv_jalr(RV_RA, RV_T1, 0);                /* helper(a0,a1) */
    } else {
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)h);
        c[n++] = rv_jalr(RV_RA, RV_T1, 0);                /* a0 = helper(a0) */
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(s_reg + rd));
        c[n++] = rv_sw(RV_A0, RV_T1, 0);                  /* reg[Rd] = a0 */
    }
    return n;
}

/* M6: compute THUMB branch condition `cond` (0x0-0xD) into RV_T0 as 0/1. Loads
 * the N/Z/C/V flag bytes into T2/T3/T4/T5; T1 is scratch. Flags are 0/1 bytes so
 * the bit-ops yield a clean 0/1. */
static int emit_cond(uint32_t *c, int n, int cond, uint8_t *nf, uint8_t *zf, uint8_t *cf, uint8_t *vf)
{
    /* load ONLY the flags this condition needs (most ARM conds use a single flag,
     * so this cuts the predicate-guard cost from 4 loads to 1 for EQ/NE/CS/CC/MI/PL/VS/VC). */
    int needN=0, needZ=0, needC=0, needV=0;
    switch (cond) {
        case 0x0: case 0x1: needZ = 1; break;
        case 0x2: case 0x3: needC = 1; break;
        case 0x4: case 0x5: needN = 1; break;
        case 0x6: case 0x7: needV = 1; break;
        case 0x8: case 0x9: needC = needZ = 1; break;
        case 0xA: case 0xB: needN = needV = 1; break;
        case 0xC: case 0xD: needN = needZ = needV = 1; break;
    }
    if (needN) { n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)nf); c[n++] = rv_lbu(RV_T2, RV_T1, 0); } /* N */
    if (needZ) { n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)zf); c[n++] = rv_lbu(RV_T3, RV_T1, 0); } /* Z */
    if (needC) { n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)cf); c[n++] = rv_lbu(RV_T4, RV_T1, 0); } /* C */
    if (needV) { n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)vf); c[n++] = rv_lbu(RV_T5, RV_T1, 0); } /* V */
    switch (cond) {
        case 0x0: c[n++] = rv_addi(RV_T0, RV_T3, 0); break;                                       /* EQ  Z */
        case 0x1: c[n++] = rv_xori(RV_T0, RV_T3, 1); break;                                       /* NE !Z */
        case 0x2: c[n++] = rv_addi(RV_T0, RV_T4, 0); break;                                       /* CS  C */
        case 0x3: c[n++] = rv_xori(RV_T0, RV_T4, 1); break;                                       /* CC !C */
        case 0x4: c[n++] = rv_addi(RV_T0, RV_T2, 0); break;                                       /* MI  N */
        case 0x5: c[n++] = rv_xori(RV_T0, RV_T2, 1); break;                                       /* PL !N */
        case 0x6: c[n++] = rv_addi(RV_T0, RV_T5, 0); break;                                       /* VS  V */
        case 0x7: c[n++] = rv_xori(RV_T0, RV_T5, 1); break;                                       /* VC !V */
        case 0x8: c[n++] = rv_xori(RV_T1, RV_T3, 1); c[n++] = rv_and(RV_T0, RV_T4, RV_T1); break; /* HI C&!Z */
        case 0x9: c[n++] = rv_xori(RV_T1, RV_T4, 1); c[n++] = rv_or (RV_T0, RV_T1, RV_T3); break; /* LS !C|Z */
        case 0xA: c[n++] = rv_xor (RV_T1, RV_T2, RV_T5); c[n++] = rv_xori(RV_T0, RV_T1, 1); break; /* GE N==V */
        case 0xB: c[n++] = rv_xor (RV_T0, RV_T2, RV_T5); break;                                   /* LT N!=V */
        case 0xC: c[n++] = rv_xor (RV_T1, RV_T2, RV_T5); c[n++] = rv_xori(RV_T1, RV_T1, 1);       /* GT !Z&(N==V) */
                  c[n++] = rv_xori(RV_T0, RV_T3, 1); c[n++] = rv_and(RV_T0, RV_T0, RV_T1); break;
        case 0xD: c[n++] = rv_xor (RV_T1, RV_T2, RV_T5); c[n++] = rv_or (RV_T0, RV_T3, RV_T1); break; /* LE Z|(N!=V) */
    }
    return n;
}

/* M6: emit a branch terminator that sets RV_A0 = next THUMB PC. Returns the new
 * word count, or -1 if `opc` is not a translatable branch (BL/BLX/SWI/undef stay
 * with the interpreter). B (fmt-18) = unconditional constant target; Bcc (fmt-16,
 * cond 0x0-0xD) = branchless select between target and the fall-through (pc+2). */
static int emit_branch(uint32_t *c, int n, uint16_t opc, uint32_t pc,
                       uint8_t *nf, uint8_t *zf, uint8_t *cf, uint8_t *vf)
{
    if ((opc & 0xF800) == 0xE000) {                       /* B (unconditional) */
        int32_t off = (int32_t)((uint32_t)(opc & 0x7FF) << 21) >> 20;   /* sign-ext 11b, <<1 */
        n = emit_li(c, n, RV_A0, pc + 4 + (uint32_t)off);
        return n;
    }
    if ((opc & 0xF000) == 0xD000) {                       /* Bcc */
        int cond = (opc >> 8) & 0xF;
        if (cond >= 0xE) return -1;                       /* 0xE undef, 0xF SWI → interpreter */
        int32_t off = (int32_t)((uint32_t)(opc & 0xFF) << 24) >> 23;    /* sign-ext 8b, <<1 */
        uint32_t target = pc + 4 + (uint32_t)off, fallth = pc + 2;
        n = emit_cond(c, n, cond, nf, zf, cf, vf);        /* taken (0/1) → t0 */
        n = emit_li(c, n, RV_A0, fallth);
        c[n++] = rv_sub(RV_T1, RV_ZERO, RV_T0);           /* t1 = -taken (0 / 0xFFFFFFFF) */
        n = emit_li(c, n, RV_A1, target - fallth);
        c[n++] = rv_and(RV_T1, RV_T1, RV_A1);             /* t1 = taken ? diff : 0 */
        c[n++] = rv_add(RV_A0, RV_A0, RV_T1);             /* a0 = fallth (+ diff if taken) */
        return n;
    }
    return -1;
}

/* Decode one THUMB opcode and emit its body (no ret) at dst[n..]. Returns the
 * new word count, or -1 if the op isn't in the translatable subset. `pc` is the
 * opcode's address (for PC-relative loads). Composes the existing emit_* (which
 * each end in a `ret`) by dropping their trailing ret. */
static int jit_emit_one(uint32_t *dst, int n, uint16_t opc, uint32_t pc)
{
    uint32_t tmp[96];
    int w = -1, rd, rs, op, sh;
    if ((opc & 0xE000) == 0x0000 && (opc & 0x1800) != 0x1800) {
        op = (opc >> 11) & 3; sh = (opc >> 6) & 0x1F; rs = (opc >> 3) & 7; rd = opc & 7;
        w = emit_shift_imm(tmp, op, rd, rs, sh, s_reg, s_nf, s_cf, s_zf);          /* fmt-1 */
    } else if ((opc & 0xF800) == 0x1800) {
        int I = (opc >> 10) & 1, sub = (opc >> 9) & 1, rni = (opc >> 6) & 7;
        rs = (opc >> 3) & 7; rd = opc & 7;
        w = emit_addsub(tmp, sub, rd, rs, I, (uint32_t)rni, s_reg, s_nf, s_cf, s_zf, s_vf); /* fmt-2 */
    } else if ((opc & 0xE000) == 0x2000) {
        op = (opc >> 11) & 3; rd = (opc >> 8) & 7;
        w = emit_alu_imm(tmp, op, rd, opc & 0xFF, s_reg, s_nf, s_cf, s_zf, s_vf);  /* fmt-3 */
    } else if ((opc & 0xFC00) == 0x4000) {
        int op4 = (opc >> 6) & 0xF; rs = (opc >> 3) & 7; rd = opc & 7;
        switch (op4) {                                                            /* fmt-4 ALU */
            case 0:  w = emit_logic(tmp, 0, rd, rs, s_reg, s_nf, s_zf); break;      /* AND */
            case 1:  w = emit_logic(tmp, 1, rd, rs, s_reg, s_nf, s_zf); break;      /* EOR */
            case 8:  w = emit_logic(tmp, 5, rd, rs, s_reg, s_nf, s_zf); break;      /* TST */
            case 12: w = emit_logic(tmp, 2, rd, rs, s_reg, s_nf, s_zf); break;      /* ORR */
            case 14: w = emit_logic(tmp, 3, rd, rs, s_reg, s_nf, s_zf); break;      /* BIC */
            case 15: w = emit_logic(tmp, 4, rd, rs, s_reg, s_nf, s_zf); break;      /* MVN */
#if VGBA_JIT_NEWOPS
            case 5:  w = emit_alu_carry(tmp, 0, rd, rs, s_reg, s_nf, s_cf, s_zf, s_vf); break; /* ADC */
            case 6:  w = emit_alu_carry(tmp, 1, rd, rs, s_reg, s_nf, s_cf, s_zf, s_vf); break; /* SBC */
            case 9:  w = emit_alu_carry(tmp, 2, rd, rs, s_reg, s_nf, s_cf, s_zf, s_vf); break; /* NEG */
            case 10: w = emit_alu_carry(tmp, 4, rd, rs, s_reg, s_nf, s_cf, s_zf, s_vf); break; /* CMP */
            case 11: w = emit_alu_carry(tmp, 3, rd, rs, s_reg, s_nf, s_cf, s_zf, s_vf); break; /* CMN */
            case 13: w = emit_mul(tmp, rd, rs, s_reg, s_nf, s_zf); break;           /* MUL */
#endif
            /* 2/3/4/7 = register-operand shifts (by Rs); 5/6/9/10/11/13 when NEWOPS off
             * → not translated → block terminates here. */
        }
    }
#if VGBA_JIT_NEWOPS
    else if ((opc & 0xFC00) == 0x4400) {                 /* fmt-5: hi-reg ADD/CMP/MOV/BX */
        int op5 = (opc >> 8) & 3;
        int rdh = (opc & 7) | (((opc >> 7) & 1) << 3);
        int rmh = ((opc >> 3) & 7) | (((opc >> 6) & 1) << 3);
        /* ADD(op5=0)/CMP(op5=1) with both operands low (H1=H2=0) are UNDEFINED in
         * VBA (thumbUI) — only translate when a hi reg is actually involved. MOV
         * (op5=2) both-low IS defined (thumb46_0). */
        int hi = (((opc >> 7) & 1) | ((opc >> 6) & 1));
        if (op5 == 3 || rdh == 15 || rmh == 15) {
            /* BX, or any PC operand/dest → block terminator. */
        } else if (op5 == 1) {
            if (hi) w = emit_alu_carry(tmp, 4, rdh, rmh, s_reg, s_nf, s_cf, s_zf, s_vf);  /* CMP */
        } else if (op5 == 0) {
            if (hi) w = emit_hireg_move(tmp, 1, rdh, rmh, s_reg);                 /* ADD (hi only) */
        } else {
            w = emit_hireg_move(tmp, 0, rdh, rmh, s_reg);                         /* MOV */
        }
    }
#endif
#if VGBA_JIT_LDST   /* load/store: corrupts state (CPUWrite side-effects use stale cpuTotalTicks/PC during a block) — disabled pending per-op state sync */
    else if ((opc & 0xF800) == 0x4800) {                 /* fmt-6: LDR Rd,[PC,#imm8] (literal) */
        rd = (opc >> 8) & 7;
        long addr = (long)(((pc + 4) & ~3u) + (uint32_t)((opc & 0xFF) << 2));
        return emit_ldst(dst, n, 0, 2, 0, rd, 0, 0, addr);
    } else if ((opc & 0xE000) == 0x6000) {               /* fmt-9: LDR/STR/LDRB/STRB [Rb,#imm5] */
        int B = (opc >> 12) & 1, L = (opc >> 11) & 1, imm5 = (opc >> 6) & 0x1F;
        rs = (opc >> 3) & 7; rd = opc & 7;
        return emit_ldst(dst, n, !L, B ? 0 : 2, 0, rd, rs, B ? imm5 : (imm5 << 2), -1);
    } else if ((opc & 0xF000) == 0x8000) {               /* fmt-10: LDRH/STRH [Rb,#imm5] */
        int L = (opc >> 11) & 1, imm5 = (opc >> 6) & 0x1F;
        rs = (opc >> 3) & 7; rd = opc & 7;
        return emit_ldst(dst, n, !L, 1, 0, rd, rs, imm5 << 1, -1);
    } else if ((opc & 0xF000) == 0x9000) {               /* fmt-11: LDR/STR [SP,#imm8] */
        int L = (opc >> 11) & 1; rd = (opc >> 8) & 7;
        return emit_ldst(dst, n, !L, 2, 0, rd, 13, (opc & 0xFF) << 2, -1);
    }
#endif
    if (w < 0) return -1;
    w--;                                  /* drop the trailing ret */
    memcpy(dst + n, tmp, (size_t)w * 4);
    return n + w;
}

/* ── M6.5 measurement: region + coverage profiler (active when g_vgba_jit==2) ──
 * Called per dispatched THUMB instr; the interpreter STILL runs it (no codegen
 * executed → zero crash risk). Tallies which region the hot THUMB runs from
 * (EWRAM 0x02 / IWRAM 0x03 / ROM 0x08-0D / other) and, per region, how many ops
 * are translatable. Answers the gating unknown: is the hot code in ROM (JIT-able
 * now) or IWRAM (needs SMC first)? jit_emit_one only writes the scratch buffer. */
unsigned g_vgba_prof_reg[4]  = { 0 };   /* dispatched instrs: EWRAM/IWRAM/ROM/other */
unsigned g_vgba_prof_xlat[4] = { 0 };   /* of those, translatable */
void vgba_jit_profile(uint32_t pc)
{
    uint32_t r = pc >> 24;
    int b = (r == 0x02) ? 0 : (r == 0x03) ? 1 : (r >= 0x08 && r <= 0x0D) ? 2 : 3;
    g_vgba_prof_reg[b]++;
    if (b == 3) return;                                   /* unknown region → can't fetch */
    if ((b == 0 && !s_ewram) || (b == 1 && !s_iwram) || (b == 2 && !s_rom)) return;
    uint32_t scratch[128];
    uint16_t opc = jit_fetch(pc);
    if (jit_emit_one(scratch, 0, opc, pc) >= 0) g_vgba_prof_xlat[b]++;
}

#define JIT_BLK_MAX_INSTR 24
#define JIT_BLK_MAX_WORDS 512

/* Build (on miss) / run a native block of consecutive translatable THUMB ops at
 * `pc` (ROM only, no R15/branch/memory ops — those terminate the block). The
 * block applies data effects only; the caller (gba.c) advances PC/prefetch/cycles
 * for the returned instruction count. Returns #instrs run, or 0 to fall back. */
int vgba_jit_thumb_dispatch(uint32_t pc)
{
    if (!s_cache || !s_arena || !s_rom) return 0;
    jit_entry_t *e = &s_cache[jit_slot(pc)];
    if (e->pc == pc) {                                /* cached (positive or negative) */
        if (!e->code) return 0;                       /* negative: known non-translatable */
        ((void (*)(void))e->code)();
        g_vgba_jit_instrs += e->ninstr;
        return e->ninstr;
    }
    /* miss → try to build a block */
    {
        uint32_t buf[JIT_BLK_MAX_WORDS];
        int n = 0, ninstr = 0;
        uint32_t p = pc;
        /* prologue (non-leaf: ops may CALL load/store helpers → preserve ra) */
        buf[n++] = rv_addi(2, 2, -16);                /* addi sp,sp,-16 */
        buf[n++] = rv_sw(RV_RA, 2, 0);                /* sw ra,0(sp)    */
        while (ninstr < JIT_BLK_MAX_INSTR && n + 96 <= JIT_BLK_MAX_WORDS) {
            uint16_t opc = jit_fetch(p);
            int nn = jit_emit_one(buf, n, opc, p);
            if (nn < 0) break;                        /* not translatable → end block */
            n = nn; ninstr++; p += 2;
        }
        if (ninstr == 0) {                            /* first op not translatable */
            e->pc = pc; e->code = NULL; e->ninstr = 0; /* NEGATIVE cache: don't re-decode */
            return 0;
        }
        buf[n++] = rv_lw(RV_RA, 2, 0);                /* lw ra,0(sp)    */
        buf[n++] = rv_addi(2, 2, 16);                 /* addi sp,sp,16  */
        buf[n++] = 0x00008067u;                       /* ret            */
        void *blk = vgba_jit_alloc(n * 4);
        if (!blk) return 0;                           /* arena full → interpreter (don't cache) */
        memcpy(blk, buf, (size_t)n * 4);
        vgba_jit_sync_icache(blk, (size_t)n * 4);
        e->pc = pc; e->code = blk; e->ninstr = (uint16_t)ninstr;
        g_vgba_blocks++;
        ((void (*)(void))e->code)();
        g_vgba_jit_instrs += e->ninstr;
        return e->ninstr;
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 *  M-A1 — ARM-mode translator (data-processing, immediate operand2)
 *  Mirrors the THUMB path: leaf blocks of consecutive AL, no-PC, translatable
 *  ARM ops applying data effects only; gba.c advances PC/prefetch/cycles. ARM &
 *  THUMB emulate the SAME ARM7TDMI, so the flag math (emit_addsub_flags) is reused
 *  verbatim. opcode index: aluop[24:21], S[20], Rn[19:16], Rd[15:12], imm @[11:0].
 * ════════════════════════════════════════════════════════════════════════════ */

/* ARM data-processing CORE: operand2 already in t2, a=reg[Rn] loaded here into t0,
 * result→t3. opc = ARM ALU op (0..15). store_lc: for logical+S, 0/1 = store that
 * constant C; -1 = leave C (LSL#0 / arithmetic / carry already stored by the reg
 * shifter). Flag math (emit_addsub_flags) is shared verbatim with THUMB. No ret. */
static int emit_arm_dp_core(uint32_t *c, int n, int opc, int rd, int rn, int S, int store_lc,
                            uint32_t *reg, uint8_t *nf, uint8_t *cf, uint8_t *zf, uint8_t *vf)
{
    int is_logical = (opc==0||opc==1||opc==8||opc==9||opc==12||opc==13||opc==14||opc==15);
    int writes     = !(opc>=8 && opc<=11);     /* TST/TEQ/CMP/CMN: flags only, no write */
    int uses_rn    = !(opc==13 || opc==15);    /* MOV/MVN ignore Rn */

    if (uses_rn) {
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rn));
        c[n++] = rv_lw(RV_T0, RV_T1, 0);                       /* t0 = a = reg[Rn] */
    }
    switch (opc) {                                             /* result → t3, b=t2 */
        case 0: case 8:  c[n++] = rv_and(RV_T3, RV_T0, RV_T2); break;                   /* AND / TST */
        case 1: case 9:  c[n++] = rv_xor(RV_T3, RV_T0, RV_T2); break;                   /* EOR / TEQ */
        case 12:         c[n++] = rv_or (RV_T3, RV_T0, RV_T2); break;                   /* ORR */
        case 13:         c[n++] = rv_addi(RV_T3, RV_T2, 0);    break;                   /* MOV  = op2 */
        case 14:         c[n++] = rv_xori(RV_T5, RV_T2, -1); c[n++] = rv_and(RV_T3, RV_T0, RV_T5); break; /* BIC a&~op2 */
        case 15:         c[n++] = rv_xori(RV_T3, RV_T2, -1); break;                     /* MVN  = ~op2 */
        case 4: case 11: c[n++] = rv_add(RV_T3, RV_T0, RV_T2); break;                   /* ADD / CMN */
        case 2: case 10: c[n++] = rv_sub(RV_T3, RV_T0, RV_T2); break;                   /* SUB / CMP */
        case 3:          c[n++] = rv_sub(RV_T3, RV_T2, RV_T0); break;                   /* RSB = op2 - a */
        case 5:          n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)cf);             /* ADC = a + op2 + C */
                         c[n++] = rv_lbu(RV_A0, RV_T1, 0);
                         c[n++] = rv_add(RV_T3, RV_T0, RV_T2);
                         c[n++] = rv_add(RV_T3, RV_T3, RV_A0); break;
        case 6:          n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)cf);             /* SBC = a - op2 - 1 + C */
                         c[n++] = rv_lbu(RV_A0, RV_T1, 0);
                         c[n++] = rv_sub(RV_T3, RV_T0, RV_T2);
                         c[n++] = rv_addi(RV_T3, RV_T3, -1);
                         c[n++] = rv_add(RV_T3, RV_T3, RV_A0); break;
        case 7:          n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)cf);             /* RSC = op2 - a - 1 + C */
                         c[n++] = rv_lbu(RV_A0, RV_T1, 0);
                         c[n++] = rv_sub(RV_T3, RV_T2, RV_T0);
                         c[n++] = rv_addi(RV_T3, RV_T3, -1);
                         c[n++] = rv_add(RV_T3, RV_T3, RV_A0); break;
    }

    if (writes) {
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rd));
        c[n++] = rv_sw(RV_T3, RV_T1, 0);                       /* reg[Rd] = res */
    }

    if (S) {
        if (is_logical) {
            if (store_lc >= 0) { c[n++] = rv_addi(RV_T4, RV_ZERO, store_lc & 1); n = emit_store_flag(c, n, cf, RV_T4); }
            /* V unchanged for logical ops */
        } else {
            int is_sub = (opc==2||opc==3||opc==6||opc==7||opc==10);   /* SUB/RSB/SBC/RSC/CMP */
            int ra = (opc==3||opc==7) ? RV_T2 : RV_T0;               /* RSB/RSC swap lhs/rhs */
            int rb = (opc==3||opc==7) ? RV_T0 : RV_T2;
            n = emit_addsub_flags(c, n, is_sub, ra, rb, RV_T3, cf, vf);
        }
        c[n++] = rv_srli(RV_T4, RV_T3, 31);  n = emit_store_flag(c, n, nf, RV_T4);   /* N */
        c[n++] = rv_sltiu(RV_T4, RV_T3, 1);  n = emit_store_flag(c, n, zf, RV_T4);   /* Z */
    }
    return n;
}

/* ARM DP, IMMEDIATE operand2 (M-A1). imm = pre-rotated constant; shc = logical
 * shifter carry (0/1, -1 = unchanged when rotate==0). Leaf block (ends in ret). */
static int emit_arm_dp_imm(uint32_t *c, int opc, int rd, int rn, uint32_t imm, int shc,
                           int S, uint32_t *reg, uint8_t *nf, uint8_t *cf, uint8_t *zf, uint8_t *vf)
{
    int n = emit_li(c, 0, RV_T2, imm);                         /* t2 = op2 = imm */
    n = emit_arm_dp_core(c, n, opc, rd, rn, S, shc, reg, nf, cf, zf, vf);
    c[n++] = rv_jalr(RV_ZERO, RV_RA, 0);
    return n;
}

/* ARM DP, REGISTER operand2 with IMMEDIATE shift (M-A2). op2 = shift(Rm,sht,sha).
 * sht: 0 LSL,1 LSR,2 ASR,3 ROR. sha 0..31 (with the ARM #0 special cases). For
 * logical+S the shifter carry-out is computed from Rm and stored to C during the
 * shift (arithmetic ops ignore it → C/V come from emit_addsub_flags). Leaf block. */
static int emit_arm_dp_reg(uint32_t *c, int opc, int rd, int rn, int rm, int sht, int sha,
                           int S, uint32_t *reg, uint8_t *nf, uint8_t *cf, uint8_t *zf, uint8_t *vf)
{
    int n = 0;
    int is_logical = (opc==0||opc==1||opc==8||opc==9||opc==12||opc==13||opc==14||opc==15);
    int want_c = (S && is_logical);            /* compute+store shifter carry? */
    int set_c  = want_c;                       /* cleared for LSL#0 (C unchanged) */

    n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(reg + rm));
    c[n++] = rv_lw(RV_T0, RV_T1, 0);           /* t0 = Rm */

    if (sht == 0) {                            /* LSL */
        if (sha == 0) { c[n++] = rv_addi(RV_T2, RV_T0, 0); set_c = 0; }      /* op2=Rm, C kept */
        else { if (want_c) { c[n++] = rv_srli(RV_T4, RV_T0, 32 - sha); c[n++] = rv_andi(RV_T4, RV_T4, 1); }
               c[n++] = rv_slli(RV_T2, RV_T0, sha); }
    } else if (sht == 1) {                     /* LSR (sha==0 → #32) */
        if (sha == 0) { if (want_c) c[n++] = rv_srli(RV_T4, RV_T0, 31); c[n++] = rv_addi(RV_T2, RV_ZERO, 0); }
        else { if (want_c) { c[n++] = rv_srli(RV_T4, RV_T0, sha - 1); c[n++] = rv_andi(RV_T4, RV_T4, 1); }
               c[n++] = rv_srli(RV_T2, RV_T0, sha); }
    } else if (sht == 2) {                     /* ASR (sha==0 → #32) */
        if (sha == 0) { if (want_c) c[n++] = rv_srli(RV_T4, RV_T0, 31); c[n++] = rv_srai(RV_T2, RV_T0, 31); }
        else { if (want_c) { c[n++] = rv_srai(RV_T4, RV_T0, sha - 1); c[n++] = rv_andi(RV_T4, RV_T4, 1); }
               c[n++] = rv_srai(RV_T2, RV_T0, sha); }
    } else {                                   /* ROR (sha==0 → RRX) */
        if (sha == 0) {                        /* RRX: op2 = (C<<31)|(Rm>>1), carry = Rm&1 */
            if (want_c) c[n++] = rv_andi(RV_T4, RV_T0, 1);
            n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)cf);
            c[n++] = rv_lbu(RV_T5, RV_T1, 0);
            c[n++] = rv_slli(RV_T5, RV_T5, 31);
            c[n++] = rv_srli(RV_T2, RV_T0, 1);
            c[n++] = rv_or(RV_T2, RV_T2, RV_T5);
        } else {                               /* ROR #sha: (Rm>>sha)|(Rm<<(32-sha)), carry = Rm bit(sha-1) */
            if (want_c) { c[n++] = rv_srli(RV_T4, RV_T0, sha - 1); c[n++] = rv_andi(RV_T4, RV_T4, 1); }
            c[n++] = rv_srli(RV_T5, RV_T0, sha);
            c[n++] = rv_slli(RV_T2, RV_T0, 32 - sha);
            c[n++] = rv_or(RV_T2, RV_T2, RV_T5);
        }
    }
    if (set_c) n = emit_store_flag(c, n, cf, RV_T4);   /* store shifter carry to C now */

    /* operand2 in t2, C handled → core leaves logical C alone (store_lc = -1) */
    n = emit_arm_dp_core(c, n, opc, rd, rn, S, -1, reg, nf, cf, zf, vf);
    c[n++] = rv_jalr(RV_ZERO, RV_RA, 0);
    return n;
}

/* Decode + emit ONE ARM instruction (cond-independent) into body[] as a standalone
 * leaf block (ends in ret). Returns words incl ret, or -1 if not translatable.
 * Handles: data-processing imm/reg-imm-shift (M-A1/A2) + single data transfer
 * LDR/STR imm-offset (M-A4). Register-shift, PC operands, writeback → terminate. */
static int arm_build_dp(uint32_t *body, uint32_t opc)
{
    uint32_t cls = opc & 0x0C000000u;

    if (cls == 0x04000000u) {                          /* bits 27:26 == 01 → single data transfer */
        int I = (opc >> 25) & 1, P = (opc >> 24) & 1, U = (opc >> 23) & 1;
        int B = (opc >> 22) & 1, W = (opc >> 21) & 1, L = (opc >> 20) & 1;
        int rn = (opc >> 16) & 0xF, rd = (opc >> 12) & 0xF;
        if (I) return -1;                              /* M-A4: immediate offset only */
        if (!P || W) return -1;                        /* pre-indexed, no writeback only */
        if (rn == 15 || rd == 15) return -1;           /* no PC base/dest */
        int imm = opc & 0xFFF;
        int off = U ? imm : -imm;
        if (off < -2047 || off > 2047) return -1;      /* addi offset range */
        int w = emit_ldst(body, 0, !L, B ? 0 : 2, 0, rd, rn, off, -1);  /* size: byte/word */
        if (w < 0) return -1;                          /* helper not registered */
        body[w++] = rv_jalr(RV_ZERO, RV_RA, 0);        /* standalone leaf ret */
        return w;
    }
    if (cls != 0x00000000u) return -1;                 /* bits 27:26 == 00 → data-processing class */
    int I = (opc >> 25) & 1;
    int aluop = (opc >> 21) & 0xF, S = (opc >> 20) & 1;
    int rn = (opc >> 16) & 0xF, rd = (opc >> 12) & 0xF;
    if (!S && aluop >= 8 && aluop <= 11) return -1;     /* op∈{TST..CMN} & S=0 → MSR/MRS, not DP */
    int writes  = !(aluop >= 8 && aluop <= 11);
    int uses_rn = !(aluop == 13 || aluop == 15);
    if (writes && rd == 15) return -1;                 /* PC dest → terminate */
    if (uses_rn && rn == 15) return -1;                /* PC src  → terminate */

    if (I) {                                           /* immediate operand2 */
        int rot = (opc >> 8) & 0xF; uint32_t imm8 = opc & 0xFF;
        uint32_t imm = rot ? ((imm8 >> (rot * 2)) | (imm8 << (32 - rot * 2))) : imm8;
        int shc = rot ? (int)(imm >> 31) : -1;
        return emit_arm_dp_imm(body, aluop, rd, rn, imm, shc, S, s_reg, s_nf, s_cf, s_zf, s_vf);
    }
    if (opc & 0x10) return -1;                          /* bit4=1 → register-shift amount (later) */
    int rm = opc & 0xF, sht = (opc >> 5) & 3, sha = (opc >> 7) & 0x1F;
    if (rm == 15) return -1;                            /* PC operand → terminate */
    return emit_arm_dp_reg(body, aluop, rd, rn, rm, sht, sha, S, s_reg, s_nf, s_cf, s_zf, s_vf);
}

/* Translate ONE ARM instruction at `pc` into dst+n. Returns new word count, or -1.
 * AL ops copy straight; conditional ops (M-A3) are wrapped in a predicate: emit_cond
 * → taken in t0, and a beq skips the whole body when the condition fails (no reg/flag
 * effects), exactly matching ARM's per-instruction conditional execution. */
static int jit_emit_one_arm(uint32_t *dst, int n, uint32_t opc, uint32_t pc)
{
    (void)pc;
    int cond = opc >> 28;
    if (cond == 0x0F) return -1;                        /* NV / unconditional-extension space */

    uint32_t body[160];
    int bw = arm_build_dp(body, opc);
    if (bw < 0) return -1;
    bw--;                                               /* drop trailing ret (chained) */

    if (cond == 0x0E) {                                 /* AL: copy body directly */
        memcpy(dst + n, body, (size_t)bw * 4);
        return n + bw;
    }
    n = emit_cond(dst, n, cond, s_nf, s_zf, s_cf, s_vf);/* taken (0/1) → t0 */
    dst[n++] = rv_beq(RV_T0, RV_ZERO, (bw + 1) * 4);    /* !taken → skip body */
    memcpy(dst + n, body, (size_t)bw * 4);
    return n + bw;
}

/* Build (on miss) / run a native leaf block of consecutive translatable ARM ops at
 * `pc`. Like vgba_jit_thumb_dispatch but 32-bit. Returns #instrs run, 0 = fall back.
 * Shares s_cache/s_arena with the THUMB path (pc tags disambiguate; ARM & THUMB
 * regions don't overlap in practice). */
int vgba_jit_arm_dispatch(uint32_t pc)
{
    if (!s_cache || !s_arena || !s_rom) return 0;
    jit_entry_t *e = &s_cache[jit_slot(pc)];
    if (e->pc == pc) {
        if (!e->code) return 0;                        /* negative cache */
        ((void (*)(void))e->code)();
        g_vgba_jit_instrs += e->ninstr;
        g_vgba_arm_instrs += e->ninstr;
        return e->ninstr;
    }
    if (s_arena_full) return 0;

    uint32_t buf[JIT_BLK_MAX_WORDS];
    int n = 0, ninstr = 0;
    uint32_t p = pc;
    buf[n++] = rv_addi(2, 2, -16);                     /* prologue (leaf, but kept for parity) */
    buf[n++] = rv_sw(RV_RA, 2, 0);
    while (ninstr < JIT_BLK_MAX_INSTR && n + 200 <= JIT_BLK_MAX_WORDS) {
        uint32_t opc = jit_fetch32(p);
        int nn = jit_emit_one_arm(buf, n, opc, p);
        if (nn < 0) break;
        n = nn; ninstr++; p += 4;
    }
    if (ninstr == 0) { e->pc = pc; e->code = NULL; e->ninstr = 0; return 0; }
    buf[n++] = rv_lw(RV_RA, 2, 0);                     /* epilogue */
    buf[n++] = rv_addi(2, 2, 16);
    buf[n++] = 0x00008067u;                            /* ret */
    void *blk = vgba_jit_alloc(n * 4);
    if (!blk) { s_arena_full = 1; return 0; }
    memcpy(blk, buf, (size_t)n * 4);
    vgba_jit_sync_icache(blk, (size_t)n * 4);
    e->pc = pc; e->code = blk; e->ninstr = (uint16_t)ninstr;
    g_vgba_blocks++;
    ((void (*)(void))e->code)();
    g_vgba_jit_instrs += e->ninstr;
    g_vgba_arm_instrs += e->ninstr;
    return e->ninstr;
}

/* M-A5: emit an ARM branch terminator that sets RV_A0 = next ARM PC, then returns
 * the new word count, or -1 if `opc` is not a translatable branch. B (cond AL) =
 * constant target; Bcc (cond 0x0-0xD) = branchless select target vs fall-through
 * (pc+4). BL (L=1, writes LR) and BX stay with the interpreter. ARM target =
 * pc + 8 + (signext24(imm24) << 2). */
static int emit_arm_branch(uint32_t *c, int n, uint32_t opc, uint32_t pc)
{
    if ((opc & 0x0E000000u) != 0x0A000000u) return -1;   /* not B/BL */
    int cond = opc >> 28;
    if (cond == 0x0F) return -1;
    int is_bl = (opc & 0x01000000u) != 0;
    int32_t off = ((int32_t)(opc << 8)) >> 6;            /* signext 24b, <<2 */
    uint32_t target = pc + 8 + (uint32_t)off;
    uint32_t fallth = pc + 4;
    if (is_bl) {                                         /* BL: LR = pc+4, then branch to target */
        if (cond != 0x0E) return -1;                     /* BLcc rare → interpreter */
        n = emit_li(c, n, RV_T0, pc + 4);
        n = emit_li(c, n, RV_T1, (uint32_t)(uintptr_t)(s_reg + 14));
        c[n++] = rv_sw(RV_T0, RV_T1, 0);                 /* reg[14] = return addr */
        n = emit_li(c, n, RV_A0, target);
        return n;
    }
    if (cond == 0x0E) {                                  /* unconditional B */
        n = emit_li(c, n, RV_A0, target);
        return n;
    }
    n = emit_cond(c, n, cond, s_nf, s_zf, s_cf, s_vf);   /* taken (0/1) → t0 */
    n = emit_li(c, n, RV_A0, fallth);
    c[n++] = rv_sub(RV_T1, RV_ZERO, RV_T0);              /* t1 = -taken (0 / 0xFFFFFFFF) */
    n = emit_li(c, n, RV_A1, target - fallth);
    c[n++] = rv_and(RV_T1, RV_T1, RV_A1);                /* t1 = taken ? diff : 0 */
    c[n++] = rv_add(RV_A0, RV_A0, RV_T1);                /* a0 = fallth + (taken?diff:0) */
    return n;
}

/* M-A5: build (on miss) and return a CHAINING ARM block at `pc`: straight-line
 * translatable ops (DP/load-store, AL or conditional) then a B/Bcc TERMINATOR that
 * returns the next ARM PC in a0. Non-branch end (non-translatable op / full) →
 * a0 = fall-through PC. *out_ninstr = #ARM instrs (incl. branch) for cycle math.
 * Returns block code (uint32_t fn(void) → next PC), NULL if entry untranslatable or
 * arena full. The gba.c hook CHAINS block→a0→block, staying in JIT across branches. */
void *vgba_jit_arm_get_block(uint32_t pc, int *out_ninstr)
{
    if (!s_cache || !s_arena || !s_rom) { *out_ninstr = 0; return NULL; }
    jit_entry_t *e = &s_cache[jit_slot(pc)];
    if (e->pc == pc) { *out_ninstr = e->ninstr; return e->code; }
    if (s_arena_full) { *out_ninstr = 0; return NULL; }

    static uint32_t buf[JIT_BLK_MAX_WORDS];
    int n = 0, ninstr = 0, terminated = 0;
    uint32_t p = pc;
    buf[n++] = rv_addi(2, 2, -16);                       /* prologue: save ra (load/store call) */
    buf[n++] = rv_sw(RV_RA, 2, 0);
    while (ninstr < JIT_BLK_MAX_INSTR && n + 200 <= JIT_BLK_MAX_WORDS) {
        uint32_t opc = jit_fetch32(p);
        int bn = emit_arm_branch(buf, n, opc, p);        /* branch terminator? */
        if (bn >= 0) { n = bn; ninstr++; terminated = 1; break; }
        int nn = jit_emit_one_arm(buf, n, opc, p);       /* data op */
        if (nn < 0) break;                               /* non-translatable → fall-through */
        n = nn; ninstr++; p += 4;
    }
    if (ninstr == 0) { e->pc = pc; e->code = NULL; e->ninstr = 0; *out_ninstr = 0; return NULL; }
    if (!terminated) n = emit_li(buf, n, RV_A0, p);      /* fall-through: a0 = next PC */
    buf[n++] = rv_lw(RV_RA, 2, 0);                       /* epilogue */
    buf[n++] = rv_addi(2, 2, 16);
    buf[n++] = 0x00008067u;                              /* ret (returns a0) */
    void *blk = vgba_jit_alloc(n * 4);
    if (!blk) { s_arena_full = 1; *out_ninstr = 0; return NULL; }
    memcpy(blk, buf, (size_t)n * 4);
    vgba_jit_sync_icache(blk, (size_t)n * 4);
    e->pc = pc; e->code = blk; e->ninstr = (uint16_t)ninstr;
    g_vgba_blocks++;
    *out_ninstr = ninstr;
    return blk;
}

/* M6: build (on miss) and return a native block for the THUMB basic block at
 * `pc` (ROM region). The block applies straight-line data effects, then a
 * TERMINATOR that leaves the next THUMB PC in a0 and returns it:
 *   - ends at a B/Bcc                      → a0 = branch result (target / pc+2);
 *   - ends at a non-translatable op / full → a0 = that PC (fall-through).
 * *out_ninstr = #THUMB instrs (incl. the branch) for the caller's cycle math.
 * Returns block code (uint32_t fn(void)), or NULL if the entry op is itself
 * non-translatable (negative-cached) or the arena is full. gba.c CHAINS: call
 * block → a0 → look up next → until it leaves ROM or hits an event boundary.
 * Live blocks APPEND in the arena after the self-test blocks (never reuse an
 * address — i-cache reuse is unsafe on this P4). */
void *vgba_jit_get_block(uint32_t pc, int *out_ninstr)
{
    if (!s_cache || !s_arena || !s_rom) { *out_ninstr = 0; return NULL; }
    jit_entry_t *e = &s_cache[jit_slot(pc)];
    if (e->pc == pc) { *out_ninstr = e->ninstr; return e->code; }  /* hit (code NULL = negative) */
    /* Once the arena is full, STOP attempting builds: re-decoding+emitting a block
     * for every uncached ROM PC each frame (only to fail the alloc) is a CPU-melting
     * retry storm (140ms). After full → cached blocks run, everything else interprets. */
    if (s_arena_full) { *out_ninstr = 0; return NULL; }

    static uint32_t buf[JIT_BLK_MAX_WORDS];   /* 2KB — off the stack (single-threaded: core0 only) */
    int n = 0, ninstr = 0, terminated = 0;
    uint32_t p = pc;
    buf[n++] = rv_addi(2, 2, -16);                       /* prologue: addi sp,sp,-16 */
    buf[n++] = rv_sw(RV_RA, 2, 0);                       /*           sw ra,0(sp)    */
    while (ninstr < JIT_BLK_MAX_INSTR && n + 96 <= JIT_BLK_MAX_WORDS) {
        uint16_t opc = jit_fetch(p);
#if VGBA_JIT_BRANCH
        int bn = emit_branch(buf, n, opc, p, s_nf, s_zf, s_cf, s_vf);  /* branch terminator? */
        if (bn >= 0) { n = bn; ninstr++; terminated = 1; break; }
#endif
#if VGBA_JIT_NOPEFFECT   /* diag: decode/count but emit NO data effect → splits exec-mechanism vs op-effects */
        { uint32_t scratch[128]; if (jit_emit_one(scratch, 0, opc, p) < 0) break; ninstr++; p += 2; continue; }
#endif
        int nn = jit_emit_one(buf, n, opc, p);           /* data op */
        if (nn < 0) break;                               /* non-translatable → fall-through */
        n = nn; ninstr++; p += 2;
    }
    if (ninstr == 0) {                                   /* entry op non-translatable */
        e->pc = pc; e->code = NULL; e->ninstr = 0; *out_ninstr = 0; return NULL;
    }
    if (!terminated) n = emit_li(buf, n, RV_A0, p);      /* fall-through: a0 = next PC */
    buf[n++] = rv_lw(RV_RA, 2, 0);                       /* epilogue: lw ra,0(sp)    */
    buf[n++] = rv_addi(2, 2, 16);                        /*           addi sp,sp,16  */
    buf[n++] = 0x00008067u;                              /*           ret (returns a0) */
    void *blk = vgba_jit_alloc(n * 4);
    /* Arena full → STOP building (set the flag, interpret the rest). gpSP-style
     * flush-when-full was tried here and FAILED on this P4: the roaming working set
     * ≫ the 64KB exec arena → it reset 80+×/window (thrash, no FPS gain) AND
     * reusing freed arena addresses for new code → stale i-cache → Instruction
     * access fault crash (64B align fixes adjacent-block line sharing, NOT reuse). */
    if (!blk) { s_arena_full = 1; *out_ninstr = 0; return NULL; }
    memcpy(blk, buf, (size_t)n * 4);
    vgba_jit_sync_icache(blk, (size_t)n * 4);
    e->pc = pc; e->code = blk; e->ninstr = (uint16_t)ninstr;
    g_vgba_blocks++;
    *out_ninstr = ninstr;
    return blk;
}

/* RISC-V: `add a0, a0, a1` ; `ret` (jalr x0,0(x1)). */
static const uint32_t k_addret[] = { 0x00B50533u, 0x00008067u };

bool vgba_jit_selftest(void)
{
    /* M0: can we emit and run code at all? */
    uint32_t *buf = heap_caps_malloc(sizeof(k_addret), MALLOC_CAP_EXEC);
    if (!buf) {
        ESP_LOGE(TAG, "M0 FAIL: no MALLOC_CAP_EXEC memory — runtime codegen unavailable");
        return false;
    }
    buf[0] = k_addret[0];
    buf[1] = k_addret[1];
    vgba_jit_sync_icache(buf, sizeof(k_addret));
    int (*fn)(int, int) = (int (*)(int, int))buf;
    int r = fn(3, 4);
    bool ok = (r == 7);
    if (ok) ESP_LOGW(TAG, "M0 PASS: runtime codegen works — f(3,4)=%d at %p. Dynarec feasible.", r, (void *)buf);
    else    ESP_LOGE(TAG, "M0 FAIL: emitted code returned %d (expected 7) at %p", r, (void *)buf);
    heap_caps_free(buf);
    if (!ok) return false;

    /* M1: arena + block-cache pipeline (emit -> register -> lookup -> run). */
    if (!vgba_jit_init(JIT_ARENA_BYTES, false)) { ESP_LOGE(TAG, "M1 FAIL: arena/cache init"); return false; }
    void *blk = vgba_jit_alloc(sizeof(k_addret));
    if (!blk) { ESP_LOGE(TAG, "M1 FAIL: arena alloc"); return false; }
    memcpy(blk, k_addret, sizeof(k_addret));
    vgba_jit_sync_icache(blk, sizeof(k_addret));

    uint32_t fake_pc = 0x08000100;
    vgba_jit_register(fake_pc, blk);
    void *got = vgba_jit_lookup(fake_pc);
    bool m1 = (got == blk && vgba_jit_lookup(fake_pc + 2) == NULL);
    if (m1) {
        int r2 = ((int (*)(int, int))got)(20, 22);
        m1 = (r2 == 42);
        ESP_LOGW(TAG, "M1 %s: arena block @%p, lookup+exec => %d (want 42)", m1 ? "PASS" : "FAIL", blk, r2);
    } else {
        ESP_LOGE(TAG, "M1 FAIL: block cache lookup mismatch");
    }
    if (!m1) { vgba_jit_release(); return false; }

    /* ── M2/M3: THUMB format-3 immediate ALU (MOV/CMP/ADD/SUB) shadow-validated
     * bit-exact vs VBA-Next's exact NEG/POS flag formulas over carry/overflow
     * boundary operands. Emit each op into TEST storage, run, compare reg+NZCV. ── */
    bool m2 = true;
    {
        static uint32_t treg[16];
        static uint8_t  tn, tc, tz, tv;
        const int tc_keep = 1, tv_keep = 1;   /* MOV must preserve these */
        const uint32_t lhss[] = { 0u, 0x7FFFFFFFu, 0x80000000u, 0xFFFFFFFFu };
        const uint32_t imms[] = { 0u, 1u };
        const char *opn[] = { "MOV", "CMP", "ADD", "SUB" };
        int tested = 0, bad = 0;
        for (int op = 0; op < 4; op++) {
            for (unsigned li = 0; li < sizeof(lhss)/4; li++) {
                for (unsigned ii = 0; ii < sizeof(imms)/4; ii++) {
                    uint32_t lhs = lhss[li], imm = imms[ii];
                    /* oracle: VBA NEG/POS exact */
                    #define NEGv(x) ((x) >> 31)
                    #define POSv(x) ((~(x)) >> 31)
                    uint32_t res, erd; int eN, eZ, eC, eV, wrote;
                    if (op == 0) { res = imm; erd = imm; eN = 0; eZ = (imm==0); eC = tc_keep; eV = tv_keep; wrote = 1; }
                    else if (op == 2) { res = lhs + imm; erd = res; wrote = 1;
                        eC = (int)((NEGv(lhs)&NEGv(imm))|(NEGv(lhs)&POSv(res))|(NEGv(imm)&POSv(res)));
                        eV = (int)((NEGv(lhs)&NEGv(imm)&POSv(res))|(POSv(lhs)&POSv(imm)&NEGv(res)));
                        eN = (int)NEGv(res); eZ = (res==0); }
                    else { res = lhs - imm; erd = (op==3)?res:lhs; wrote = (op==3);
                        eC = (int)((NEGv(lhs)&POSv(imm))|(NEGv(lhs)&POSv(res))|(POSv(imm)&POSv(res)));
                        eV = (int)((NEGv(lhs)&POSv(imm)&POSv(res))|(POSv(lhs)&NEGv(imm)&NEGv(res)));
                        eN = (int)NEGv(res); eZ = (res==0); }
                    memset(treg, 0, sizeof(treg)); treg[3] = lhs;
                    tn = 0xAA; tz = 0xAA; tc = tc_keep; tv = tv_keep;
                    uint32_t *blk = vgba_jit_alloc(40 * 4);
                    if (!blk) { ESP_LOGE(TAG, "M2 FAIL: arena alloc"); m2 = false; break; }
                    int words = emit_alu_imm(blk, op, 3, imm, treg, &tn, &tc, &tz, &tv);
                    vgba_jit_sync_icache(blk, (size_t)words * 4);
                    ((void (*)(void))blk)();
                    bool ok = (treg[3] == (wrote ? erd : lhs)) &&
                              (tn==eN) && (tz==eZ) && (tc==eC) && (tv==eV);
                    tested++; if (!ok) { bad++; m2 = false;
                        ESP_LOGE(TAG, "M3 BAD %s r3=0x%08x #0x%x -> r3=0x%08x N%d Z%d C%d V%d (want r3=0x%08x N%d Z%d C%d V%d)",
                                 opn[op], (unsigned)lhs, (unsigned)imm, (unsigned)treg[3], tn,tz,tc,tv,
                                 (unsigned)(wrote?erd:lhs), eN,eZ,eC,eV);
                    }
                    #undef NEGv
                    #undef POSv
                }
                if (!m2) break;
            }
            if (!m2) break;
        }
        ESP_LOGW(TAG, "M3 %s: format-3 ALU (MOV/CMP/ADD/SUB imm) %d/%d bit-exact",
                 m2 ? "PASS" : "FAIL", tested - bad, tested);
    }

    /* ── M3b: THUMB format-2 ADD/SUB Rd,Rs,<Rn|#imm3> shadow-validated bit-exact.
     * dst=r3, src(Rs)=r2, rhs reg(Rn)=r1 or imm. Same NZCV oracle as M3a. ── */
    {
        /* no arena reset — unique addresses (reuse is i-cache-unsafe on this P4) */
        static uint32_t treg[16];
        static uint8_t  tn, tc, tz, tv;
        const uint32_t lhss[] = { 0u, 0x7FFFFFFFu, 0x80000000u };
        const uint32_t rhss[] = { 1u, 0x80000000u, 0xFFFFFFFFu };
        int tested = 0, bad = 0;
        for (int is_sub = 0; is_sub < 2; is_sub++) {
            for (int imm_mode = 0; imm_mode < 2; imm_mode++) {
                for (unsigned li = 0; li < sizeof(lhss)/4; li++) {
                    for (unsigned ri = 0; ri < sizeof(rhss)/4; ri++) {
                        uint32_t lhs = lhss[li], rhs = rhss[ri];
                        #define NEGv(x) ((x) >> 31)
                        #define POSv(x) ((~(x)) >> 31)
                        uint32_t res = is_sub ? (lhs - rhs) : (lhs + rhs);
                        int eN = (int)NEGv(res), eZ = (res==0), eC, eV;
                        if (!is_sub) {
                            eC = (int)((NEGv(lhs)&NEGv(rhs))|(NEGv(lhs)&POSv(res))|(NEGv(rhs)&POSv(res)));
                            eV = (int)((NEGv(lhs)&NEGv(rhs)&POSv(res))|(POSv(lhs)&POSv(rhs)&NEGv(res)));
                        } else {
                            eC = (int)((NEGv(lhs)&POSv(rhs))|(NEGv(lhs)&POSv(res))|(POSv(rhs)&POSv(res)));
                            eV = (int)((NEGv(lhs)&POSv(rhs)&POSv(res))|(POSv(lhs)&NEGv(rhs)&NEGv(res)));
                        }
                        #undef NEGv
                        #undef POSv
                        memset(treg, 0, sizeof(treg));
                        treg[2] = lhs; treg[1] = rhs; treg[3] = 0xDEADBEEF;
                        tn = 0xAA; tz = 0xAA; tc = 0xAA; tv = 0xAA;
                        uint32_t *blk = vgba_jit_alloc(48 * 4);
                        if (!blk) { ESP_LOGE(TAG, "M3b FAIL: arena alloc"); m2 = false; break; }
                        int words = emit_addsub(blk, is_sub, 3, 2, imm_mode, imm_mode ? rhs : 1,
                                                treg, &tn, &tc, &tz, &tv);
                        vgba_jit_sync_icache(blk, (size_t)words * 4);
                        ((void (*)(void))blk)();
                        bool ok = (treg[3]==res) && (tn==eN) && (tz==eZ) && (tc==eC) && (tv==eV);
                        tested++; if (!ok) { bad++; m2 = false;
                            ESP_LOGE(TAG, "M3b BAD %s%s r2=0x%08x r1=0x%08x -> r3=0x%08x N%d Z%d C%d V%d (want 0x%08x N%d Z%d C%d V%d)",
                                     is_sub?"SUB":"ADD", imm_mode?"i":"r", (unsigned)lhs, (unsigned)rhs,
                                     (unsigned)treg[3], tn,tz,tc,tv, (unsigned)res, eN,eZ,eC,eV);
                        }
                    }
                    if (!m2) break;
                }
                if (!m2) break;
            }
            if (!m2) break;
        }
        ESP_LOGW(TAG, "M3b %s: format-2 ADD/SUB (reg & imm3) %d/%d bit-exact",
                 m2 ? "PASS" : "FAIL", tested - bad, tested);
    }

    /* ── M3c-logic: format-4 logical AND/EOR/ORR/BIC/MVN/TST (N/Z only). ── */
    {
        static uint32_t treg[16];
        static uint8_t  tn, tc, tz, tv;
        const uint32_t a[] = { 0u, 0xFFFFFFFFu };
        const uint32_t b[] = { 0u, 0xF0F0F0F0u };
        const char *opn[] = { "AND","EOR","ORR","BIC","MVN","TST" };
        int tested = 0, bad = 0;
        for (int op = 0; op < 6; op++)
        for (unsigned ai = 0; ai < sizeof(a)/4 && m2; ai++)
        for (unsigned bi = 0; bi < sizeof(b)/4 && m2; bi++) {
            uint32_t rd = a[ai], rs = b[bi], res;
            switch (op) { case 0: case 5: res = rd & rs; break; case 1: res = rd ^ rs; break;
                          case 2: res = rd | rs; break; case 3: res = rd & ~rs; break;
                          default: res = ~rs; break; }
            int eN = (int)(res >> 31), eZ = (res == 0), wrote = (op != 5);
            memset(treg, 0, sizeof(treg)); treg[3] = rd; treg[2] = rs;
            tn = 0xAA; tz = 0xAA; tc = 7; tv = 9;          /* C/V must stay */
            uint32_t *blk = vgba_jit_alloc(40 * 4);
            if (!blk) { ESP_LOGE(TAG, "M3c FAIL alloc"); m2 = false; break; }
            int words = emit_logic(blk, op, 3, 2, treg, &tn, &tz);
            vgba_jit_sync_icache(blk, (size_t)words * 4);
            ((void (*)(void))blk)();
            bool ok = (treg[3] == (wrote ? res : rd)) && (tn==eN) && (tz==eZ) && (tc==7) && (tv==9);
            tested++; if (!ok) { bad++; m2 = false;
                ESP_LOGE(TAG, "M3c BAD %s rd=0x%08x rs=0x%08x -> 0x%08x N%d Z%d C%d V%d (want 0x%08x N%d Z%d)",
                         opn[op], (unsigned)rd, (unsigned)rs, (unsigned)treg[3], tn,tz,tc,tv,
                         (unsigned)(wrote?res:rd), eN, eZ); }
        }
        ESP_LOGW(TAG, "M3c-logic %s: format-4 logical (AND/EOR/ORR/BIC/MVN/TST) %d/%d bit-exact",
                 m2 ? "PASS" : "FAIL", tested - bad, tested);
    }

    /* ── M3c-shift: format-1 LSL/LSR/ASR Rd,Rs,#imm5 (N/Z/C, V kept; #0 cases). ── */
    {
        static uint32_t treg[16];
        static uint8_t  tn, tc, tz, tv;
        const uint32_t rm[] = { 1u, 0x80000000u, 0xFFFFFFFFu };
        const int shifts[] = { 0, 1, 31 };
        const char *opn[] = { "LSL","LSR","ASR" };
        int tested = 0, bad = 0;
        for (int op = 0; op < 3; op++)
        for (unsigned vi = 0; vi < sizeof(rm)/4 && m2; vi++)
        for (unsigned si = 0; si < sizeof(shifts)/sizeof(int) && m2; si++)
        for (int cin = 0; cin < 2 && m2; cin++) {
            uint32_t v = rm[vi]; int sh = shifts[si];
            uint32_t res; int eC = cin;                    /* C preserved by default */
            if (op == 0) { if (sh == 0) { res = v; } else { eC = (int)((v >> (32 - sh)) & 1); res = v << sh; } }
            else if (op == 1) { if (sh) { eC = (int)((v >> (sh - 1)) & 1); res = v >> sh; } else { res = 0; eC = (int)(v >> 31); } }
            else { if (sh) { eC = (int)(((int32_t)v >> (sh - 1)) & 1); res = (uint32_t)((int32_t)v >> sh); }
                   else { if (v & 0x80000000u) { res = 0xFFFFFFFFu; eC = 1; } else { res = 0; eC = 0; } } }
            int eN = (int)(res >> 31), eZ = (res == 0);
            memset(treg, 0, sizeof(treg)); treg[2] = v;
            tn = 0xAA; tz = 0xAA; tc = (uint8_t)cin; tv = 5;   /* V must stay; C starts = cin */
            uint32_t *blk = vgba_jit_alloc(40 * 4);
            if (!blk) { ESP_LOGE(TAG, "M3c FAIL alloc"); m2 = false; break; }
            int words = emit_shift_imm(blk, op, 3, 2, sh, treg, &tn, &tc, &tz);
            vgba_jit_sync_icache(blk, (size_t)words * 4);
            ((void (*)(void))blk)();
            bool ok = (treg[3]==res) && (tn==eN) && (tz==eZ) && (tc==eC) && (tv==5);
            tested++; if (!ok) { bad++; m2 = false;
                ESP_LOGE(TAG, "M3c BAD %s#%d v=0x%08x cin%d -> 0x%08x N%d Z%d C%d V%d (want 0x%08x N%d Z%d C%d)",
                         opn[op], sh, (unsigned)v, cin, (unsigned)treg[3], tn,tz,tc,tv, (unsigned)res, eN,eZ,eC); }
        }
        ESP_LOGW(TAG, "M3c-shift %s: format-1 LSL/LSR/ASR imm %d/%d bit-exact",
                 m2 ? "PASS" : "FAIL", tested - bad, tested);
    }

    /* ── M3d: format-4 ADC/SBC/NEG/CMN/CMP(reg) — full N/Z/C/V vs VBA's exact
     * NEG/POS carry+overflow (ADC/SBC carry-in makes res<lhs wrong → must match
     * the 3-term formulas). Rd=r3, Rs=r2; C_FLAG input varied for ADC/SBC. ── */
    {
        static uint32_t treg[16];
        static uint8_t  tn, tc, tz, tv;
        const uint32_t lhss[] = { 0x7FFFFFFFu, 0x80000000u, 0xFFFFFFFFu };
        const uint32_t rhss[] = { 1u, 0xFFFFFFFFu };
        const char *opn[] = { "ADC","SBC","NEG","CMN","CMP" };
        int tested = 0, bad = 0;
        #define NEGv(x) ((x) >> 31)
        #define POSv(x) ((~(x)) >> 31)
        for (int op = 0; op <= 4 && m2; op++) {
            int adc_sbc = (op == 0 || op == 1);
            int is_sub  = (op == 1 || op == 2 || op == 4);
            int writes  = (op != 3 && op != 4);
            int rcnt = (op == 2) ? 1 : 2;            /* NEG: single operand (Rs) */
            for (int li = 0; li < 3 && m2; li++)
            for (int ri = 0; ri < rcnt && m2; ri++) {
                int ccnt = adc_sbc ? 2 : 1;
                for (int cin = 0; cin < ccnt && m2; cin++) {
                    uint32_t a, b, res;
                    if (op == 2) { a = 0; b = lhss[li]; res = a - b; }              /* NEG: 0 - Rs */
                    else { a = lhss[li]; b = rhss[ri];
                           res = !adc_sbc ? (is_sub ? a - b : a + b)
                                          : (is_sub ? a - b - 1u + (uint32_t)cin
                                                    : a + b + (uint32_t)cin); }
                    int eN = (int)NEGv(res), eZ = (res == 0), eC, eV;
                    if (!is_sub) {
                        eC = (int)((NEGv(a)&NEGv(b))|(NEGv(a)&POSv(res))|(NEGv(b)&POSv(res)));
                        eV = (int)((NEGv(a)&NEGv(b)&POSv(res))|(POSv(a)&POSv(b)&NEGv(res)));
                    } else {
                        eC = (int)((NEGv(a)&POSv(b))|(NEGv(a)&POSv(res))|(POSv(b)&POSv(res)));
                        eV = (int)((NEGv(a)&POSv(b)&POSv(res))|(POSv(a)&NEGv(b)&NEGv(res)));
                    }
                    uint32_t e_rd = writes ? res : a;          /* CMN/CMP leave Rd; others write res */
                    memset(treg, 0, sizeof(treg));
                    treg[3] = a; treg[2] = b;                  /* NEG: a=0 so treg[3] just gets res */
                    tn = 0xAA; tz = 0xAA; tc = (uint8_t)cin; tv = 0xAA;
                    uint32_t *blk = vgba_jit_alloc(80 * 4);
                    if (!blk) { ESP_LOGE(TAG, "M3d FAIL alloc"); m2 = false; break; }
                    int words = emit_alu_carry(blk, op, 3, 2, treg, &tn, &tc, &tz, &tv);
                    vgba_jit_sync_icache(blk, (size_t)words * 4);
                    ((void (*)(void))blk)();
                    bool ok = (treg[3] == e_rd) && (tn==eN) && (tz==eZ) && (tc==eC) && (tv==eV);
                    tested++; if (!ok) { bad++; m2 = false;
                        ESP_LOGE(TAG, "M3d BAD %s%s Rd=0x%08x Rs=0x%08x cin%d -> 0x%08x N%d Z%d C%d V%d (want 0x%08x N%d Z%d C%d V%d)",
                                 opn[op], adc_sbc?(cin?"+c":"  "):"  ", (unsigned)a, (unsigned)b, cin,
                                 (unsigned)treg[3], tn,tz,tc,tv, (unsigned)e_rd, eN,eZ,eC,eV); }
                }
            }
        }
        #undef NEGv
        #undef POSv
        ESP_LOGW(TAG, "M3d %s: format-4 ADC/SBC/NEG/CMN/CMP %d/%d bit-exact",
                 m2 ? "PASS" : "FAIL", tested - bad, tested);
    }

    /* ── M3e: format-4 MUL (Rd = Rs*Rd; N/Z only, C/V kept). ── */
    {
        static uint32_t treg[16];
        static uint8_t  tn, tc, tz, tv;
        const uint32_t av[] = { 0x00000003u, 0x80000000u, 0xFFFFFFFFu };
        const uint32_t bv[] = { 0x00000002u, 0x7FFFFFFFu };
        int tested = 0, bad = 0;
        for (unsigned ai = 0; ai < 3 && m2; ai++)
        for (unsigned bi = 0; bi < 2 && m2; bi++) {
            uint32_t rd = av[ai], rs = bv[bi], res = rs * rd;
            int eN = (int)(res >> 31), eZ = (res == 0);
            memset(treg, 0, sizeof(treg)); treg[3] = rd; treg[2] = rs;
            tn = 0xAA; tz = 0xAA; tc = 7; tv = 9;             /* C/V must stay */
            uint32_t *blk = vgba_jit_alloc(40 * 4);
            if (!blk) { ESP_LOGE(TAG, "M3e FAIL alloc"); m2 = false; break; }
            int words = emit_mul(blk, 3, 2, treg, &tn, &tz);
            vgba_jit_sync_icache(blk, (size_t)words * 4);
            ((void (*)(void))blk)();
            bool ok = (treg[3] == res) && (tn==eN) && (tz==eZ) && (tc==7) && (tv==9);
            tested++; if (!ok) { bad++; m2 = false;
                ESP_LOGE(TAG, "M3e BAD MUL 0x%08x*0x%08x -> 0x%08x N%d Z%d C%d V%d (want 0x%08x N%d Z%d)",
                         (unsigned)rd, (unsigned)rs, (unsigned)treg[3], tn,tz,tc,tv, (unsigned)res, eN, eZ); }
        }
        ESP_LOGW(TAG, "M3e %s: format-4 MUL %d/%d bit-exact", m2 ? "PASS" : "FAIL", tested - bad, tested);
    }

    /* ── M3f: format-5 hi-reg ADD/MOV (no flags) + CMP (full NZCV, no write),
     * reg-reg subset (both ≤ r14; PC operands/BX terminate the block). Uses a hi
     * index (r10) as the source to exercise reg[8..14] addressing. ── */
    {
        static uint32_t treg[16];
        static uint8_t  tn, tc, tz, tv;
        int tested = 0, bad = 0;
        const struct { int is_add; uint32_t rd0, rm0; } mv[] = {
            { 1, 0x00000005u, 0x00000007u }, { 1, 0xFFFFFFFFu, 0x00000001u },
            { 0, 0xDEADBEEFu, 0x12345678u },
        };
        for (unsigned i = 0; i < 3 && m2; i++) {           /* ADD/MOV: flags must stay */
            uint32_t e = mv[i].is_add ? (mv[i].rd0 + mv[i].rm0) : mv[i].rm0;
            memset(treg, 0, sizeof(treg)); treg[3] = mv[i].rd0; treg[10] = mv[i].rm0;
            tn = 1; tz = 1; tc = 1; tv = 1;
            uint32_t *blk = vgba_jit_alloc(24 * 4);
            if (!blk) { ESP_LOGE(TAG, "M3f FAIL alloc"); m2 = false; break; }
            int words = emit_hireg_move(blk, mv[i].is_add, 3, 10, treg);
            vgba_jit_sync_icache(blk, (size_t)words * 4);
            ((void (*)(void))blk)();
            bool ok = (treg[3] == e) && tn == 1 && tz == 1 && tc == 1 && tv == 1;
            tested++; if (!ok) { bad++; m2 = false;
                ESP_LOGE(TAG, "M3f BAD %s r3=0x%08x r10=0x%08x -> 0x%08x N%d Z%d C%d V%d (want 0x%08x flags 1111)",
                         mv[i].is_add ? "ADD" : "MOV", (unsigned)mv[i].rd0, (unsigned)mv[i].rm0,
                         (unsigned)treg[3], tn,tz,tc,tv, (unsigned)e); }
        }
        #define NEGv(x) ((x) >> 31)
        #define POSv(x) ((~(x)) >> 31)
        const uint32_t cl[] = { 0x80000000u, 5u }, cr[] = { 1u, 5u };
        for (unsigned ai = 0; ai < 2 && m2; ai++)          /* CMP hi-reg: full NZCV, no write */
        for (unsigned bi = 0; bi < 2 && m2; bi++) {
            uint32_t a = cl[ai], b = cr[bi], res = a - b;
            int eN = (int)NEGv(res), eZ = (res == 0);
            int eC = (int)((NEGv(a)&POSv(b))|(NEGv(a)&POSv(res))|(POSv(b)&POSv(res)));
            int eV = (int)((NEGv(a)&POSv(b)&POSv(res))|(POSv(a)&NEGv(b)&NEGv(res)));
            memset(treg, 0, sizeof(treg)); treg[3] = a; treg[10] = b;
            tn = 0xAA; tz = 0xAA; tc = 0xAA; tv = 0xAA;
            uint32_t *blk = vgba_jit_alloc(80 * 4);
            if (!blk) { ESP_LOGE(TAG, "M3f FAIL alloc"); m2 = false; break; }
            int words = emit_alu_carry(blk, 4, 3, 10, treg, &tn, &tc, &tz, &tv);
            vgba_jit_sync_icache(blk, (size_t)words * 4);
            ((void (*)(void))blk)();
            bool ok = (treg[3] == a) && tn == eN && tz == eZ && tc == eC && tv == eV;
            tested++; if (!ok) { bad++; m2 = false;
                ESP_LOGE(TAG, "M3f BAD CMP 0x%08x,0x%08x -> N%d Z%d C%d V%d w0x%08x (want N%d Z%d C%d V%d)",
                         (unsigned)a, (unsigned)b, tn,tz,tc,tv, (unsigned)treg[3], eN,eZ,eC,eV); }
        }
        #undef NEGv
        #undef POSv
        ESP_LOGW(TAG, "M3f %s: format-5 hi-reg ADD/MOV/CMP %d/%d bit-exact",
                 m2 ? "PASS" : "FAIL", tested - bad, tested);
    }

    /* ── M6: branch terminators. Build a Bcc block ending in a branchless select
     * and verify the RETURNED next-PC (a0) for every condition × flag combo vs a C
     * oracle; plus one unconditional B. Blocks are leaf (no ret save) → just the
     * emit_branch body + ret. ── */
    {
        static uint8_t tn, tz, tc, tv;
        const uint32_t PC = 0x08000100u;
        struct { int N, Z, C, V; } combo[] = { {0,0,0,0},{1,1,1,1},{1,0,1,0},{0,1,0,1} };
        int tested = 0, bad = 0;
        for (int cond = 0; cond <= 0xD && m2; cond++)
        for (unsigned ci = 0; ci < 4 && m2; ci++) {
            int N = combo[ci].N, Z = combo[ci].Z, C = combo[ci].C, V = combo[ci].V;
            int taken;
            switch (cond) {
                case 0x0: taken = Z; break;            case 0x1: taken = !Z; break;
                case 0x2: taken = C; break;            case 0x3: taken = !C; break;
                case 0x4: taken = N; break;            case 0x5: taken = !N; break;
                case 0x6: taken = V; break;            case 0x7: taken = !V; break;
                case 0x8: taken = C && !Z; break;      case 0x9: taken = !C || Z; break;
                case 0xA: taken = (N == V); break;     case 0xB: taken = (N != V); break;
                case 0xC: taken = !Z && (N == V); break; default: taken = Z || (N != V); break;
            }
            uint16_t opc = 0xD000 | (cond << 8) | 0x10;     /* off8=0x10 → +0x20 */
            uint32_t target = PC + 4 + 0x20, fallth = PC + 2;
            uint32_t want = taken ? target : fallth;
            tn = (uint8_t)N; tz = (uint8_t)Z; tc = (uint8_t)C; tv = (uint8_t)V;
            uint32_t *blk = vgba_jit_alloc(48 * 4);
            if (!blk) { ESP_LOGE(TAG, "M6 FAIL alloc"); m2 = false; break; }
            int w = emit_branch(blk, 0, opc, PC, &tn, &tz, &tc, &tv);
            blk[w++] = 0x00008067u;                         /* ret (a0 = next PC) */
            vgba_jit_sync_icache(blk, (size_t)w * 4);
            uint32_t got = ((uint32_t (*)(void))blk)();
            tested++; if (got != want) { bad++; m2 = false;
                ESP_LOGE(TAG, "M6 BAD Bcc cond=%X NZCV=%d%d%d%d -> 0x%08x (want 0x%08x, taken=%d)",
                         cond, N,Z,C,V, (unsigned)got, (unsigned)want, taken); }
        }
        /* unconditional B: +0x40 */
        if (m2) {
            uint16_t opc = 0xE000 | 0x20;                   /* off11=0x20 → +0x40 */
            uint32_t want = PC + 4 + 0x40;
            uint32_t *blk = vgba_jit_alloc(8 * 4);
            if (blk) {
                int w = emit_branch(blk, 0, opc, PC, &tn, &tz, &tc, &tv);
                blk[w++] = 0x00008067u;
                vgba_jit_sync_icache(blk, (size_t)w * 4);
                uint32_t got = ((uint32_t (*)(void))blk)();
                tested++; if (got != want) { bad++; m2 = false;
                    ESP_LOGE(TAG, "M6 BAD B -> 0x%08x (want 0x%08x)", (unsigned)got, (unsigned)want); }
            }
        }
        ESP_LOGW(TAG, "M6 %s: branch terminators (Bcc×14conds + B) %d/%d correct",
                 m2 ? "PASS" : "FAIL", tested - bad, tested);
    }

    /* Self-test done. The translators + branch terminators are validated. JIT
     * integration = chaining inner loop in gba.c calling vgba_jit_get_block (M6).
     * Default ships DISARMED (interpreter). VGBA_JIT_ARM arms the chaining JIT for
     * measurement; VGBA_PROFILE arms the measurement-only region profiler. */
#if VGBA_JIT_ARM
    /* Separate self-test from live: FREE the self-test arena+cache, alloc a FRESH
     * full arena for gameplay (self-test ate ~50KB of 60KB → only 20 blocks fit).
     * Live blocks now get the whole arena. The self-test is done, so reusing the
     * freed addresses is safe (every live block is sync_icache'd before it runs). */
    vgba_jit_release();
    vgba_jit_init(JIT_ARENA_BYTES, false);   /* fast internal exec SRAM (PSRAM was too slow) */
    s_arena_full = 0;
    g_vgba_jit = 1;     /* arm the chaining JIT on ROM */
    ESP_LOGW(TAG, "VGBA_JIT_ARM: chaining JIT armed (g_vgba_jit=1); %uKB internal live arena",
             (unsigned)(JIT_ARENA_BYTES / 1024));
#elif VGBA_PROFILE
    vgba_jit_release();
    g_vgba_jit = 2;     /* measurement-only: per-instr region+coverage profiler, no codegen run */
    ESP_LOGW(TAG, "VGBA_PROFILE: profiler armed (g_vgba_jit=2) — interpreter still runs every op");
#else
    vgba_jit_release();
#endif
    return m1 && m2;
}

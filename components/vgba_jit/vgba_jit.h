#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* tab5-vgbanext dynamic recompiler (ARM7TDMI -> RISC-V), built incrementally.
 * M0: runtime-codegen self-test + exec arena + block cache. Later milestones add
 * the THUMB/ARM dispatch + translation (see DYNAREC_PLAN.md). The RISC-V emitter,
 * arena, cache and I-cache-flush infrastructure are adapted from the proven mGBA
 * dynarec in tab5-gba (gba_jit.c); only GBA-state access is retargeted to
 * VBA-Next's global `bus.reg[]`. */

/* M0/M1 self-test: prove MALLOC_CAP_EXEC + I-cache flush + RISC-V emission work
 * on this board, then the arena/cache emit->register->lookup->run pipeline.
 * Returns true if codegen is usable (dynarec feasible). */
bool vgba_jit_selftest(void);

/* Allocate/free the exec arena + block cache. arena_bytes sizes the exec arena
 * (big for the self-test's unique-address blocks; small for live dispatch). */
bool  vgba_jit_init(size_t arena_bytes, bool psram);  /* psram=true → MB-scale exec arena in PSRAM */
void  vgba_jit_release(void);
void  vgba_jit_reset(void);

/* Bump-allocate executable space (4-byte aligned); NULL if arena full. */
void *vgba_jit_alloc(size_t bytes);

/* Block cache: index by THUMB pc>>1. */
void  vgba_jit_register(uint32_t pc, void *code);
void *vgba_jit_lookup(uint32_t pc);

/* Make freshly-written code at [buf,buf+len) visible to instruction fetch. */
void  vgba_jit_sync_icache(void *buf, size_t len);

/* ── Dispatch (called from VBA-Next's THUMB execute loop in gba.c) ──
 * g_vgba_jit: 1 = JIT dispatch active. vgba_jit_thumb_dispatch(): looks up /
 * builds a native block for the current THUMB PC (global bus.reg[15]); returns 1
 * if it ran a block (caller continues its loop) or 0 to fall back to the
 * interpreter. M1: always returns 0 (proves the hook is correct + side-effect
 * free). gba.c references these via plain extern (no component REQUIRES edge, to
 * avoid a cycle — both archives are in the final app link). */
extern int g_vgba_jit;                  /* 0=off, 1=chaining JIT, 2=measurement profiler */
int  vgba_jit_thumb_dispatch(uint32_t pc);

/* M6 chaining block builder. Returns a native block (uint32_t fn(void) → next
 * THUMB PC) for the basic block at pc, building on miss; NULL if the entry op is
 * non-translatable (negative-cached) or the arena is full. *out_ninstr = #THUMB
 * instrs (incl. terminating branch) for the caller's cycle accounting. gba.c
 * chains blocks until execution leaves ROM or reaches a cpu event boundary. */
void *vgba_jit_get_block(uint32_t pc, int *out_ninstr);

/* M6.5 profiler (g_vgba_jit==2): tally per-region THUMB-instr counts + how many
 * are translatable, to locate the hot code (ROM vs IWRAM). Interpreter still runs
 * every op; no native block is executed. Counters drained per window by vgba_run. */
void vgba_jit_profile(uint32_t pc);
extern unsigned g_vgba_prof_reg[4];     /* EWRAM / IWRAM / ROM / other */
extern unsigned g_vgba_prof_xlat[4];    /* of those, translatable */

/* M2/M4: gba.c registers the addresses of its file-static CPU state + the ROM
 * buffer so the emitter can bake addresses in and the block builder can decode
 * THUMB opcodes. reg = &bus.reg[0].I (stride 4, reg[15]=PC); n/c/z/v =
 * &N/C/Z/V_FLAG (1-byte bools); rom/rom_mask let dispatch read opcodes as
 * rom[(pc & rom_mask) & ~1]. Called once from CPUInit. */
void vgba_jit_set_state(uint32_t *reg, uint8_t *n, uint8_t *c, uint8_t *z, uint8_t *v,
                        uint8_t *rom, uint32_t rom_mask, uint8_t *iwram, uint8_t *ewram);

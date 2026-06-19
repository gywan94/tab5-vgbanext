#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_common.h"
#include "file_browser.h"
#include "vgba_run.h"
#include "vgba_jit.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_cpu.h"
#include "esp_private/esp_clk.h"

static const char *TAG = "vgba_tab5";

/* ── Tier-3 feasibility prototype: measure dispatch-mechanism cost on the P4 ──
 * Same per-"instruction" work (read 2 regs, ALU, write reg, set N/Z), dispatched
 * two ways: (A) function-pointer table + global scalar state (today's interpreter
 * style) vs (B) computed-goto threaded + scalar state in locals. The register file
 * is memory-resident in BOTH (dynamic indexing forbids register allocation). The
 * A/B ratio is the realistic upper bound of a threaded-interpreter rewrite. */
#define DB_OPS 4096
static uint32_t g_dbops[DB_OPS];

static uint32_t gA_reg[16];
static uint8_t  gA_nf, gA_zf;
static void hA_add(uint32_t op){ uint32_t r=gA_reg[(op>>4)&15]+gA_reg[(op>>8)&15]; gA_reg[op&15]=r; gA_nf=r>>31; gA_zf=(r==0); }
static void hA_sub(uint32_t op){ uint32_t r=gA_reg[(op>>4)&15]-gA_reg[(op>>8)&15]; gA_reg[op&15]=r; gA_nf=r>>31; gA_zf=(r==0); }
static void hA_and(uint32_t op){ uint32_t r=gA_reg[(op>>4)&15]&gA_reg[(op>>8)&15]; gA_reg[op&15]=r; gA_nf=r>>31; gA_zf=(r==0); }
static void hA_or (uint32_t op){ uint32_t r=gA_reg[(op>>4)&15]|gA_reg[(op>>8)&15]; gA_reg[op&15]=r; gA_nf=r>>31; gA_zf=(r==0); }
static void (*tblA[4])(uint32_t) = { hA_add, hA_sub, hA_and, hA_or };

static uint32_t bench_funcptr(int reps){
    for(int i=0;i<16;i++) gA_reg[i]=i*7u+1;
    for(int r=0;r<reps;r++)
        for(int i=0;i<DB_OPS;i++){ uint32_t op=g_dbops[i]; tblA[(op>>12)&3](op); }
    return gA_reg[0]+gA_nf+gA_zf;
}

static uint32_t bench_threaded(int reps){
    uint32_t reg[16]; for(int i=0;i<16;i++) reg[i]=i*7u+1;   /* register file STILL in memory */
    uint8_t nf=0, zf=0;                                       /* scalars in locals → registers */
    static const void* tbl[4]={&&Ladd,&&Lsub,&&Land,&&Lor};
    long total=(long)reps*DB_OPS, idx=0;
    uint32_t op=g_dbops[0];
    goto *tbl[(op>>12)&3];
  Ladd: { uint32_t r=reg[(op>>4)&15]+reg[(op>>8)&15]; reg[op&15]=r; nf=r>>31; zf=(r==0);
          if(++idx>=total){goto done;} op=g_dbops[idx&(DB_OPS-1)]; goto *tbl[(op>>12)&3]; }
  Lsub: { uint32_t r=reg[(op>>4)&15]-reg[(op>>8)&15]; reg[op&15]=r; nf=r>>31; zf=(r==0);
          if(++idx>=total){goto done;} op=g_dbops[idx&(DB_OPS-1)]; goto *tbl[(op>>12)&3]; }
  Land: { uint32_t r=reg[(op>>4)&15]&reg[(op>>8)&15]; reg[op&15]=r; nf=r>>31; zf=(r==0);
          if(++idx>=total){goto done;} op=g_dbops[idx&(DB_OPS-1)]; goto *tbl[(op>>12)&3]; }
  Lor:  { uint32_t r=reg[(op>>4)&15]|reg[(op>>8)&15]; reg[op&15]=r; nf=r>>31; zf=(r==0);
          if(++idx>=total){goto done;} op=g_dbops[idx&(DB_OPS-1)]; goto *tbl[(op>>12)&3]; }
  done: return reg[0]+nf+zf;
}

static void dispatch_bench(void){
    for(int i=0;i<DB_OPS;i++) g_dbops[i] = (uint32_t)i*2654435761u;   /* well-mixed → unpredictable handler+regs */
    int reps=3000; long ops=(long)reps*DB_OPS;
    int mhz = esp_clk_cpu_freq()/1000000;
    volatile uint32_t sink=0;
    uint32_t t0=esp_cpu_get_cycle_count(); sink+=bench_funcptr(reps); uint32_t ca=esp_cpu_get_cycle_count()-t0;
    t0=esp_cpu_get_cycle_count(); sink+=bench_threaded(reps); uint32_t cb=esp_cpu_get_cycle_count()-t0;
    double pa=(double)ca/ops, pb=(double)cb/ops;
    ESP_LOGW(TAG,"DISPATCH-BENCH @%dMHz: funcptr=%.1f cyc/op  threaded=%.1f cyc/op  speedup=%.2fx  sink=%lu",
             mhz, pa, pb, pa/pb, (unsigned long)sink);
}

/* TEMP MEMBENCH: measure P4 PSRAM vs internal-SRAM read speed, compare to GBA. */
static void mem_bench(void)
{
    int mhz = esp_clk_cpu_freq() / 1000000;
    const size_t CNT = 1u << 20;          /* 1M words = 4 MiB working set (>> L2) */
    const uint32_t STEP = 521;            /* prime, coprime to 2^20 -> one big cycle */

    volatile uint32_t *ps = heap_caps_malloc(CNT * 4, MALLOC_CAP_SPIRAM | MALLOC_CAP_32BIT);
    if (!ps) { ESP_LOGE(TAG, "MEMBENCH PSRAM alloc fail"); return; }
    const size_t ICNT = 8u << 10;         /* 8K words = 32 KiB internal (optional ref) */
    volatile uint32_t *in = heap_caps_malloc(ICNT * 4, MALLOC_CAP_INTERNAL | MALLOC_CAP_32BIT);

    /* pointer-chase fill: arr[i] = (i+STEP)%CNT -> data-dependent random walk */
    for (size_t i = 0; i < CNT;  i++) ps[i] = (uint32_t)((i + STEP) % CNT);
    if (in) for (size_t i = 0; i < ICNT; i++) in[i] = (uint32_t)((i + STEP) % ICNT);

    size_t free_int = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    ESP_LOGW(TAG, "MEMBENCH free internal SRAM = %u KB", (unsigned)(free_int / 1024));

    volatile uint32_t sink = 0;
    uint32_t t0, t1;

    /* 1) PSRAM sequential read bandwidth (L2 prefetch helps) */
    t0 = esp_cpu_get_cycle_count();
    uint32_t acc = 0;
    for (size_t i = 0; i < CNT; i++) acc += ps[i];
    t1 = esp_cpu_get_cycle_count();
    sink += acc;
    double seq_cyc = (double)(t1 - t0);
    double seq_mbs = (double)(CNT * 4) / (seq_cyc / (mhz * 1e6)) / 1e6;
    ESP_LOGW(TAG, "MEMBENCH PSRAM seq:  %.0f MB/s  (%.2f cyc/word)", seq_mbs, seq_cyc / CNT);

    /* 2) PSRAM random pointer-chase latency (cache-defeated, true latency) */
    uint32_t idx = 0;
    t0 = esp_cpu_get_cycle_count();
    for (size_t i = 0; i < CNT; i++) idx = ps[idx];
    t1 = esp_cpu_get_cycle_count();
    sink += idx;
    double pr_cyc = (double)(t1 - t0) / CNT;
    ESP_LOGW(TAG, "MEMBENCH PSRAM rand: %.1f cyc/acc = %.1f ns/acc", pr_cyc, pr_cyc / mhz * 1000.0);

    /* 3) internal SRAM random pointer-chase latency (reference) */
    if (in) {
        idx = 0;
        t0 = esp_cpu_get_cycle_count();
        for (size_t i = 0; i < CNT; i++) idx = in[idx % ICNT];
        t1 = esp_cpu_get_cycle_count();
        sink += idx;
        double ir_cyc = (double)(t1 - t0) / CNT;
        ESP_LOGW(TAG, "MEMBENCH SRAM  rand: %.1f cyc/acc = %.1f ns/acc", ir_cyc, ir_cyc / mhz * 1000.0);
    }

    ESP_LOGW(TAG, "MEMBENCH cpu=%dMHz sink=%lu", mhz, (unsigned long)sink);
    heap_caps_free((void *)ps);
    if (in) heap_caps_free((void *)in);
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== GBA Tab5 (VBA-Next) Starting ===");

    app_init();

    (void)mem_bench;   /* TEMP probe, not run during JIT test */
    dispatch_bench();  /* Tier-3 feasibility: dispatch-mechanism cost on P4 */

    /* M0: prove runtime codegen works on this board (dynarec self-test). The JIT
     * is at M1 — dispatch always misses → interpreter, so it's behavior-neutral. */
    vgba_jit_selftest();

    file_browser_init("/sd");

    /* TEMP (IPROF): auto-run the flash ROM to profile instruction mix. Restore browser after. */
    for (;;) {
        ESP_LOGI(TAG, "IPROF auto-run flash:romdata");
        vgba_run("flash:romdata");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

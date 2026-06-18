#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_common.h"
#include "file_browser.h"
#include "vgba_run.h"
#include "vgba_jit.h"
#include "esp_log.h"

static const char *TAG = "vgba_tab5";

void app_main(void)
{
    ESP_LOGI(TAG, "=== GBA Tab5 (VBA-Next) Starting ===");

    app_init();

    /* M0: prove runtime codegen works on this board (dynarec self-test). The JIT
     * is at M1 — dispatch always misses → interpreter, so it's behavior-neutral. */
    vgba_jit_selftest();

    file_browser_init("/sd");

    /* Pick a ROM on the SD card, run it; when the game exits (MENU) return to
     * the browser to pick the next one. */
    for (;;) {
        char path[512];
        if (file_browser_run(path, sizeof(path)) == 0) {
            ESP_LOGI(TAG, "Launching: %s", path);
            vgba_run(path);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

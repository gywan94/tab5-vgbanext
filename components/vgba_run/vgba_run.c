/*
 * vgba_run.c — VBA-Next (libretro/vba-next) glue for the M5Stack Tab5.
 *
 * VBA-Next is a libretro core. Instead of pulling in a full libretro frontend,
 * we compile the core + its libretro.c and implement just the handful of
 * callbacks libretro.c needs (environment / video / audio / input). The core is
 * driven one frame at a time:
 *
 *     poll input  ->  retro_run()  ->  (core calls systemDrawScreen ->
 *     video_cb, and systemOnWriteDataToSoundBuffer -> audio_batch_cb during
 *     the run)  ->  we blit the frame (PPA 3x scale + 270deg rot via DSI) and
 *     push the resampled audio.
 *
 * Video is RGB565 240x160 (stride 256 px); audio is stereo at 32 kHz, resampled
 * to the 48 kHz codec rate. Layout: game upright in the top region, on-screen
 * gamepad below (portrait), same as the mGBA port. MENU exits to the launcher.
 *
 * First cut is a single-core serial loop for correctness; the emulate/display
 * split that the mGBA port uses can be layered on later (pix is a complete
 * frame, so a display task would just ping-pong copies of it).
 */
#include "vgba_run.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_cpu.h"
#include "esp_log.h"

#include "odroid_audio.h"
#include "odroid_input.h"
#include "odroid_display.h"
#include "odroid_sdcard.h"
#include "odroid_vpad.h"

#include <libretro.h>

#include "esp_partition.h"
#include "miniz.h"   /* ESP-IDF bundles tinfl (raw deflate) in esp_rom */

static const char *TAG = "VGBA_RUN";

#define GBA_W 240
#define GBA_H 160
#define GBA_PITCH_PX 256                 /* VBA-Next pix stride (PIX_BUFFER_SCREEN_WIDTH) */
#define AUDIO_SAMPLE_RATE 48000          /* ES8388 hardware rate */
#define CORE_AUDIO_RATE   32000          /* gba_init() calls soundSetSampleRate(32000) */
#define AUDIO_FRAMES_CAP  4096           /* per-frame stereo capacity (generous) */

/* GBA save RAM lives in libretro.c's libretro_save_buf, surfaced via
 * retro_get_memory_data(RETRO_MEMORY_SAVE_RAM). */

/* Dual-core pipeline: core 0 emulates frame N+1 into buffer s_cur while a core-1
 * task blits frame N (PPA scale/rot + DSI) and pushes its audio. Two buffers
 * ping-pong; a free-buffer queue gives backpressure so emulation never overruns
 * display by more than one frame. Hides the ~6 ms display+audio cost behind the
 * ~33 ms emulate; it does NOT speed up emulation itself. */
#define NB 2                             /* ping-pong buffer count */
static uint16_t *s_fb[NB]      = { NULL };/* compacted 240x160 RGB565 frames  */
static int16_t  *s_aud_acc[NB] = { NULL };/* accumulated core audio (32 kHz)  */
static volatile int s_aud_n[NB] = { 0 }; /* stereo frames in s_aud_acc[i]     */
static int16_t  *s_aud_rs   = NULL;      /* resampled-to-48k output (core1)   */
static uint8_t  *s_rom      = NULL;      /* whole ROM in PSRAM                */
static volatile int  s_cur  = 0;         /* buffer core0 emulates into now     */
static volatile bool s_quit = false;

static QueueHandle_t     s_free_q = NULL; /* buffer indices ready to emulate into */
static QueueHandle_t     s_disp_q = NULL; /* finished frames ready to display      */
static SemaphoreHandle_t s_disp_done = NULL;
static volatile int s_cur_fps = 0, s_cur_cpu = 0; /* for periodic log */
/* VOL press is requested on core0 but APPLIED on core1 (the display task), so all
 * ES8388 codec access (audio submit + volume/mute) stays on one core — avoids a
 * cross-core race on the codec handle that froze the device. */
static volatile int s_vol_req = 0;
static unsigned s_jit_instrs_prev = 0;   /* for per-window JIT throughput log */
static int s_frameskip = 0;              /* 0=draw every frame, N=render 1 of N+1 */
extern void SetFrameskip(int code);      /* vba-next core (USE_FRAME_SKIP) */

#ifndef VGBA_FRAME_LIMIT
#define VGBA_FRAME_LIMIT 1   /* cap emulation to the GBA's native ~59.73 fps. With the JIT
                              * armed the emulator can run >60fps, producing audio faster than
                              * the 48kHz I2S sink consumes it → the GB APU blip buffer overflows
                              * (broken audio). Pacing the loop to native rate keeps audio synced
                              * and turns the surplus speed into idle/power saving. Harmless when
                              * running <60fps (the no-JIT shipped config never hits the cap). */
#endif
/* GBA frame = 280896 CPU cycles @ 16.777216 MHz → 16743 us (59.727 fps). */
#define VGBA_FRAME_US 16743

#ifndef VGBA_FPS_OVERLAY
#define VGBA_FPS_OVERLAY 1   /* draw the live FPS into the frame's corner */
#endif
/* The frame is scaled 3× + rotated 270° by the PPA, so the SOURCE top-right maps
 * to the DISPLAYED top-left. Flip these anchors if it lands in another corner. */
#define FPS_ANCHOR_RIGHT  0   /* 0 = source left edge (= display top-left, portrait/no-rotation) */
#define FPS_ANCHOR_BOTTOM 0   /* 0 = source top edge                                            */

/* Compact 8×8 font for digits 0-9 (one byte per row, MSB = left pixel). */
static const uint8_t k_fps_font[10][8] = {
    {0x3C,0x66,0x6E,0x76,0x66,0x66,0x3C,0x00}, {0x18,0x38,0x18,0x18,0x18,0x18,0x7E,0x00},
    {0x3C,0x66,0x06,0x0C,0x18,0x30,0x7E,0x00}, {0x3C,0x66,0x06,0x1C,0x06,0x66,0x3C,0x00},
    {0x0C,0x1C,0x3C,0x6C,0x7E,0x0C,0x0C,0x00}, {0x7E,0x60,0x7C,0x06,0x06,0x66,0x3C,0x00},
    {0x1C,0x30,0x60,0x7C,0x66,0x66,0x3C,0x00}, {0x7E,0x06,0x0C,0x18,0x30,0x30,0x30,0x00},
    {0x3C,0x66,0x66,0x3C,0x66,0x66,0x3C,0x00}, {0x3C,0x66,0x66,0x3E,0x06,0x0C,0x38,0x00},
};

/* Draw `fps` (0-99) into the 240×160 RGB565 frame at the chosen corner, 2× scale
 * (16px source → ~48px on screen), white digits on a black box for legibility. */
static void draw_fps_overlay(uint16_t *fb, int fps)
{
    if (!fb) return;
    if (fps < 0) fps = 0; else if (fps > 99) fps = 99;
    int d[2], n = 0;
    if (fps >= 10) d[n++] = (fps / 10) % 10;
    d[n++] = fps % 10;

    const int G = 8, SC = 2, GAP = 1, gw = G * SC;   /* glyph 16px at 2× */
    int total = n * gw + (n - 1) * GAP, marg = 3;
    int x0 = FPS_ANCHOR_RIGHT  ? (GBA_W - marg - total) : marg;
    int y0 = FPS_ANCHOR_BOTTOM ? (GBA_H - marg - gw)    : marg;

    for (int by = -1; by <= gw; by++)                /* background box */
        for (int bx = -1; bx <= total; bx++) {
            int px = x0 + bx, py = y0 + by;
            if (px >= 0 && px < GBA_W && py >= 0 && py < GBA_H) fb[py * GBA_W + px] = 0x0000;
        }
    for (int i = 0; i < n; i++) {
        int gx = x0 + i * (gw + GAP);
        for (int row = 0; row < G; row++) {
            uint8_t bits = k_fps_font[d[i]][row];
            for (int col = 0; col < G; col++) {
                if (!(bits & (0x80 >> col))) continue;
                for (int sy = 0; sy < SC; sy++)
                    for (int sx = 0; sx < SC; sx++) {
                        int px = gx + col * SC + sx, py = y0 + row * SC + sy;
                        if (px >= 0 && px < GBA_W && py >= 0 && py < GBA_H) fb[py * GBA_W + px] = 0xFFFF;
                    }
            }
        }
    }
}

/* ─── libretro key bits (must match libretro.c binds[] -> J bit order) ─── */

/* ─── Input ─────────────────────────────────────────────────────────────
 * libretro.c's update_input() queries input_cb(0, JOYPAD, 0, id) per button.
 * We snapshot the gamepad once per frame in poll_cb and answer from it. */
static odroid_gamepad_state s_gp;

#ifndef VGBA_AUTOINPUT
#define VGBA_AUTOINPUT 1   /* 1 = mash START then A on an 80-frame cycle to blow past
                            * title/intro screens into THUMB-heavy gameplay (measurement
                            * builds; pair with VGBA_PROFILE). idf.py -DVGBA_AUTOINPUT=1 */
#endif
#if VGBA_AUTOINPUT
static unsigned s_autoframe = 0;
#endif

static void rl_poll(void)
{
    odroid_input_gamepad_read(&s_gp);
#if VGBA_AUTOINPUT
    s_autoframe++;
#endif
}

static int16_t rl_input_state(unsigned port, unsigned device, unsigned index, unsigned id)
{
    (void)port; (void)device; (void)index;
    int16_t v = 0;
    switch (id) {
        case RETRO_DEVICE_ID_JOYPAD_A:      v = s_gp.values[ODROID_INPUT_A];      break;
        case RETRO_DEVICE_ID_JOYPAD_B:      v = s_gp.values[ODROID_INPUT_B];      break;
        case RETRO_DEVICE_ID_JOYPAD_SELECT: v = s_gp.values[ODROID_INPUT_SELECT]; break;
        case RETRO_DEVICE_ID_JOYPAD_START:  v = s_gp.values[ODROID_INPUT_START];  break;
        case RETRO_DEVICE_ID_JOYPAD_RIGHT:  v = s_gp.values[ODROID_INPUT_RIGHT];  break;
        case RETRO_DEVICE_ID_JOYPAD_LEFT:   v = s_gp.values[ODROID_INPUT_LEFT];   break;
        case RETRO_DEVICE_ID_JOYPAD_UP:     v = s_gp.values[ODROID_INPUT_UP];     break;
        case RETRO_DEVICE_ID_JOYPAD_DOWN:   v = s_gp.values[ODROID_INPUT_DOWN];   break;
        case RETRO_DEVICE_ID_JOYPAD_R:      v = s_gp.values[ODROID_INPUT_R];      break;
        case RETRO_DEVICE_ID_JOYPAD_L:      v = s_gp.values[ODROID_INPUT_L];      break;
        default:                            v = 0;                                break;
    }
#if VGBA_AUTOINPUT
    unsigned f = s_autoframe % 80;
    if (id == RETRO_DEVICE_ID_JOYPAD_START && f < 8)             v = 1;
    if (id == RETRO_DEVICE_ID_JOYPAD_A     && f >= 40 && f < 48) v = 1;
#endif
    return v;
}

/* ─── Video ─────────────────────────────────────────────────────────────
 * Called once per frame from systemDrawScreen(). data is the core's pix buffer
 * (stride GBA_PITCH_PX px); compact the 240 valid columns into s_fb. */
static void rl_video(const void *data, unsigned width, unsigned height, size_t pitch)
{
    uint16_t *dst = s_fb[s_cur];
    if (!data || !dst) return;
    const uint16_t *src = (const uint16_t *)data;
    int spitch = (int)(pitch / 2);          /* bytes -> pixels */
    int w = width  > GBA_W ? GBA_W : (int)width;
    int h = height > GBA_H ? GBA_H : (int)height;
    /* Compact the 240 valid columns (stride-256 → packed-240) here on core0. A/B
     * tested moving this to core1 (contiguous copy + in-place compact) → no FPS
     * gain (the strided vs contiguous copy is a wash on PSRAM), reverted. */
    for (int y = 0; y < h; y++)
        memcpy(dst + (size_t)y * GBA_W, src + (size_t)y * spitch, (size_t)w * 2);
}

/* ─── Audio ─────────────────────────────────────────────────────────────
 * Called from sound.c with stereo int16 at CORE_AUDIO_RATE. Accumulate; the
 * main loop resamples to 48k and submits after the frame. */
static size_t rl_audio_batch(const int16_t *data, size_t frames)
{
    int16_t *acc = s_aud_acc[s_cur];
    if (acc && frames > 0) {
        int cur = s_aud_n[s_cur];
        int room = AUDIO_FRAMES_CAP - cur;
        int n = (int)frames > room ? room : (int)frames;
        if (n > 0) {
            memcpy(acc + (size_t)cur * 2, data, (size_t)n * 2 * sizeof(int16_t));
            s_aud_n[s_cur] = cur + n;
        }
    }
    return frames;
}

/* Linear resample CORE_AUDIO_RATE -> 48k, phase carried across frames. */
static double s_rs_phase = 0.0;
static int audio_resample_48k(int16_t *dst, int dstcap, const int16_t *src, int n)
{
    double step = (double)CORE_AUDIO_RATE / (double)AUDIO_SAMPLE_RATE;
    double p = s_rs_phase;
    int out = 0;
    while (p < n && out < dstcap) {
        int i = (int)p, i2 = (i + 1 < n) ? i + 1 : i;
        double f = p - i;
        dst[out * 2]     = (int16_t)(src[i * 2]     + (int)((src[i2 * 2]     - src[i * 2])     * f));
        dst[out * 2 + 1] = (int16_t)(src[i * 2 + 1] + (int)((src[i2 * 2 + 1] - src[i * 2 + 1]) * f));
        out++; p += step;
    }
    s_rs_phase = p - n;
    if (s_rs_phase < 0) s_rs_phase = 0;
    return out;
}

/* ─── Environment ────────────────────────────────────────────────────────
 * Minimal: accept RGB565, decline everything else (defaults are fine). */
static bool rl_environ(unsigned cmd, void *data)
{
    switch (cmd) {
        case RETRO_ENVIRONMENT_SET_PIXEL_FORMAT: {
            const enum retro_pixel_format *fmt = (const enum retro_pixel_format *)data;
            return fmt && *fmt == RETRO_PIXEL_FORMAT_RGB565;
        }
        case RETRO_ENVIRONMENT_GET_CAN_DUPE:
            if (data) *(bool *)data = false;
            return true;
        default:
            return false;   /* no log iface, no vfs, no variables, no system dir */
    }
}

/* libretro.c references filestream_vfs_init() but only calls it after a
 * successful GET_VFS_INTERFACE (which we decline). Provide a stub so we don't
 * have to compile the whole libretro-common streams/vfs subsystem. */
struct retro_vfs_interface_info;
void filestream_vfs_init(const struct retro_vfs_interface_info *info) { (void)info; }

/* ─── Minimal in-memory ZIP extractor (raw tinfl, same as the mGBA port) ── */
static inline uint16_t zrd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t zrd32(const uint8_t *p) {
    return (uint32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24));
}
/* Minimal in-memory ZIP extractor. HARDENED: every offset/length taken from the
 * (untrusted) zip is bounds-checked against [zip, zip+zlen) before use, sizes are
 * capped, and the "stored" output is sized to csize (not the header's usize) so a
 * spoofed usize<csize can't overflow the heap. Picks the largest .gba entry. */
static uint8_t *gba_unzip_rom(const uint8_t *zip, size_t zlen, long *out_size)
{
    const uint32_t ZIP_MAX = 0x2000000u + 0x20000u;   /* GBA 32MB cap + headroom; reject absurd sizes */
    if (zlen < 22) return NULL;
    long scan_start = (long)zlen - 22, scan_min = scan_start - 65536;
    if (scan_min < 0) scan_min = 0;
    long eocd = -1;
    for (long i = scan_start; i >= scan_min; i--)               /* eocd ≤ zlen-22 → eocd+19 < zlen */
        if (zrd32(zip + i) == 0x06054b50) { eocd = i; break; }
    if (eocd < 0) return NULL;
    uint32_t cd_off = zrd32(zip + eocd + 16);
    uint16_t n_ent  = zrd16(zip + eocd + 10);
    if ((size_t)cd_off + 46 > zlen) return NULL;                /* central directory must be inside */

    long best_off = -1; uint32_t best_csize = 0, best_usize = 0, best_method = 0, best_pick = 0;
    long p = (long)cd_off;
    for (int e = 0; e < n_ent && p >= 0 && p + 46 <= (long)zlen; e++) {
        if (zrd32(zip + p) != 0x02014b50) break;
        uint16_t method = zrd16(zip + p + 10);
        uint32_t csize  = zrd32(zip + p + 20);
        uint32_t usize  = zrd32(zip + p + 24);
        uint16_t nlen   = zrd16(zip + p + 28);
        uint16_t elen   = zrd16(zip + p + 30);
        uint16_t clen   = zrd16(zip + p + 32);
        uint32_t lho    = zrd32(zip + p + 42);
        if (p + 46 + (long)nlen > (long)zlen) break;            /* filename must be inside */
        const char *name = (const char *)(zip + p + 46);
        bool is_gba = nlen >= 4 && strncasecmp(name + nlen - 4, ".gba", 4) == 0;
        uint32_t pick = (is_gba ? 0x40000000u : 0) + usize;
        if (pick > best_pick) { best_pick = pick; best_off = (long)lho; best_csize = csize; best_usize = usize; best_method = method; }
        p += 46 + nlen + elen + clen;
    }
    if (best_off < 0) return NULL;

    /* Output capacity is per-method: stored → csize (the data IS the output);
     * deflate → usize. Reject zero/absurd sizes before allocating. */
    uint32_t out_cap = (best_method == 0) ? best_csize : best_usize;
    if (out_cap == 0 || out_cap > ZIP_MAX || best_csize > ZIP_MAX) return NULL;

    /* Local header + the compressed data it points to must lie fully inside the zip. */
    if ((size_t)best_off + 30 > zlen) return NULL;
    const uint8_t *lh = zip + best_off;
    if (zrd32(lh) != 0x04034b50) return NULL;
    uint16_t lnlen = zrd16(lh + 26), lelen = zrd16(lh + 28);
    size_t data_off = (size_t)best_off + 30 + lnlen + lelen;
    if (data_off > zlen || data_off + best_csize > zlen) return NULL;   /* data range in bounds */
    const uint8_t *cdata = zip + data_off;

    uint8_t *out = heap_caps_malloc(out_cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!out) return NULL;
    if (best_method == 0) {                       /* stored: copy exactly csize into a csize buffer */
        memcpy(out, cdata, best_csize);
        *out_size = (long)best_csize;
    } else {                                       /* deflate: tinfl bounded by out_cap=usize */
        size_t got = tinfl_decompress_mem_to_mem(out, best_usize, cdata, best_csize, 0);
        if (got == TINFL_DECOMPRESS_MEM_TO_MEM_FAILED) { heap_caps_free(out); return NULL; }
        *out_size = (long)got;
    }
    return out;
}

/* Read a ROM from a flash partition (path "flash:<name>") into PSRAM. The actual
 * ROM size is the partition contents minus the trailing 0xFF erased padding. */
static long load_rom_flash(const char *part_name)
{
    const esp_partition_t *p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                                        ESP_PARTITION_SUBTYPE_ANY, part_name);
    if (!p) { ESP_LOGE(TAG, "partition '%s' not found", part_name); return 0; }

    const void *map = NULL;
    esp_partition_mmap_handle_t h = 0;
    if (esp_partition_mmap(p, 0, p->size, ESP_PARTITION_MMAP_DATA, &map, &h) != ESP_OK) {
        ESP_LOGE(TAG, "mmap '%s' failed", part_name); return 0;
    }
    const uint8_t *fp = (const uint8_t *)map;
    /* Trim trailing erased flash (0xFF) to get the real ROM size. */
    long sz = (long)p->size;
    while (sz > 0 && fp[sz - 1] == 0xFF) sz--;
    if (sz & 1) sz++;
    if (sz < 0xC0 || ((fp[0xB2] != 0x96))) {  /* 0xB2 == 0x96: GBA fixed value sanity check */
        ESP_LOGW(TAG, "flash ROM sanity: size=%ld byte0xB2=0x%02x (continuing)", sz, fp[0xB2]);
    }
    if (sz <= 0) { esp_partition_munmap(h); ESP_LOGE(TAG, "flash ROM empty"); return 0; }

    s_rom = heap_caps_malloc(sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_rom) { esp_partition_munmap(h); ESP_LOGE(TAG, "ROM PSRAM alloc failed (%ld)", sz); return 0; }
    memcpy(s_rom, fp, sz);
    esp_partition_munmap(h);
    ESP_LOGI(TAG, "ROM from flash '%s': %ld bytes", part_name, sz);
    return sz;
}

/* Read the ROM (.gba or .zip) into PSRAM. Returns size, 0 on failure. */
static long load_rom(const char *rom_path)
{
    if (strncmp(rom_path, "flash:", 6) == 0)
        return load_rom_flash(rom_path + 6);

    FILE *f = fopen(rom_path, "rb");
    if (!f) { ESP_LOGE(TAG, "open ROM failed: %s", rom_path); return 0; }
    fseek(f, 0, SEEK_END); long file_size = ftell(f); fseek(f, 0, SEEK_SET);

    const char *ext = strrchr(rom_path, '.');
    bool is_zip = ext && strcasecmp(ext, ".zip") == 0;
    long rom_size = 0;

    if (is_zip) {
        uint8_t *zbuf = heap_caps_malloc(file_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!zbuf) { fclose(f); ESP_LOGE(TAG, "zip buf alloc failed (%ld)", file_size); return 0; }
        size_t zread = fread(zbuf, 1, file_size, f);
        fclose(f);
        s_rom = gba_unzip_rom(zbuf, zread, &rom_size);
        heap_caps_free(zbuf);
        if (!s_rom) { ESP_LOGE(TAG, "zip: no .gba inside"); return 0; }
        ESP_LOGI(TAG, "ROM from zip: %ld bytes", rom_size);
    } else {
        rom_size = file_size;
        s_rom = heap_caps_malloc(rom_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_rom) { fclose(f); ESP_LOGE(TAG, "ROM PSRAM alloc failed (%ld)", rom_size); return 0; }
        size_t got = fread(s_rom, 1, rom_size, f);
        fclose(f);
        ESP_LOGI(TAG, "ROM loaded: %u bytes", (unsigned)got);
    }
    return rom_size;
}

/* Build the .sav path next to the ROM. */
static void sav_path(const char *rom_path, char *out, size_t cap)
{
    snprintf(out, cap, "%s", rom_path);
    char *dot = strrchr(out, '.');
    if (dot) strcpy(dot, ".sav"); else strncat(out, ".sav", cap - strlen(out) - 1);
}

/* Core-1 task: blit finished frames (PPA 3x + DSI) and push their audio. Owns
 * the panel + codec + the cross-frame resample phase for the duration. */
static void vgba_display_task(void *arg)
{
    (void)arg;
    int idx;
    int fc = 0;
    while (xQueueReceive(s_disp_q, &idx, portMAX_DELAY) == pdTRUE) {
        if (idx < 0) break;                                  /* sentinel: stop */
        /* Apply a pending VOL change here (core1) so it's serialized with the
         * audio submit below — all codec access on one core. */
        if (s_vol_req) { s_vol_req = 0; odroid_audio_volume_change(); }
        ili9341_write_frame_rgb565_custom(s_fb[idx], GBA_W, GBA_H, 3.0f, false);
        /* Re-assert the on-screen gamepad once the game is up. The one-shot draw at
         * launch can be lost when a fast-booting game's first blit races it; redrawing
         * here (same task as the game blit → never concurrent) guarantees it shows.
         * The game blit only touches the top region, so this bottom-region redraw is
         * needed just a couple of times, not every frame. */
        if (fc == 30 || fc == 90) odroid_vpad_draw();
        fc++;
        int n = s_aud_n[idx];
        if (n > 0 && s_aud_rs) {
            int rsn = audio_resample_48k(s_aud_rs, AUDIO_FRAMES_CAP * 2, s_aud_acc[idx], n);
            int off = 0;
            while (rsn > 0) {
                int c = rsn > 1024 ? 1024 : rsn;
                odroid_audio_submit(s_aud_rs + (size_t)off * 2, c);
                off += c; rsn -= c;
            }
        }
        xQueueSend(s_free_q, &idx, portMAX_DELAY);           /* hand buffer back */
    }
    xSemaphoreGive(s_disp_done);
    vTaskDelete(NULL);
}

void vgba_run(const char *rom_path)
{
    ESP_LOGI(TAG, "vgba_run: %s", rom_path);
    s_quit = false;
    s_cur = 0;
    s_rs_phase = 0.0;
    s_frameskip = 0;            /* default OFF (render every frame). SELECT+R cycles 0→1→2. */
    SetFrameskip(0);

    for (int i = 0; i < NB; i++) {
        s_fb[i]      = heap_caps_calloc(1, GBA_W * GBA_H * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        /* PSRAM (was INTERNAL): leave internal SRAM for the JIT exec arena. */
        s_aud_acc[i] = heap_caps_malloc(AUDIO_FRAMES_CAP * 2 * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        s_aud_n[i]   = 0;
        if (!s_fb[i] || !s_aud_acc[i]) { ESP_LOGE(TAG, "fb/audio alloc failed"); goto cleanup; }
    }
    s_aud_rs = heap_caps_malloc(AUDIO_FRAMES_CAP * 2 * 2 * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_aud_rs) { ESP_LOGE(TAG, "audio rs alloc failed"); goto cleanup; }

    if (odroid_sdcard_open("/sd") != ESP_OK) { ESP_LOGE(TAG, "SD open failed"); goto cleanup; }

    long rom_size = load_rom(rom_path);
    if (!rom_size) goto cleanup;

    /* Wire up libretro callbacks, then bring up the core. */
    retro_set_environment(rl_environ);
    retro_set_video_refresh(rl_video);
    retro_set_audio_sample_batch(rl_audio_batch);
    retro_set_input_poll(rl_poll);
    retro_set_input_state(rl_input_state);
    retro_init();

    struct retro_game_info game = {
        .path = rom_path,
        .data = s_rom,
        .size = (size_t)rom_size,
        .meta = NULL,
    };
    if (!retro_load_game(&game)) { ESP_LOGE(TAG, "retro_load_game failed"); goto cleanup_core; }

    /* The core copied the ROM into its own buffer (CPULoadRomData → memcpy), so our
     * s_rom copy is now dead. Free it immediately: a large/zipped ROM otherwise holds
     * ~2× the ROM in PSRAM for the whole session, which starved the 900KB on-screen
     * gamepad buffer → no gamepad on big ROMs. */
    if (s_rom) { heap_caps_free(s_rom); s_rom = NULL; }

    /* Restore save RAM from .sav, if present, into the core's save buffer. */
    {
        char save[512]; sav_path(rom_path, save, sizeof(save));
        void  *sram = retro_get_memory_data(RETRO_MEMORY_SAVE_RAM);
        size_t scap = retro_get_memory_size(RETRO_MEMORY_SAVE_RAM);
        FILE *sf = fopen(save, "rb");
        if (sf && sram && scap) {
            size_t rd = fread(sram, 1, scap, sf);
            ESP_LOGI(TAG, "loaded save: %u bytes", (unsigned)rd);
        }
        if (sf) fclose(sf);
    }

    odroid_audio_init(AUDIO_SAMPLE_RATE);

    /* Portrait layout: game upright top, on-screen gamepad bottom. */
    odroid_input_xy_menu_disable = true;
    display_set_emu_portrait(true);
    ili9341_write_frame_rgb565_custom(NULL, GBA_W, GBA_H, 3.0f, false);
    odroid_vpad_draw();

    odroid_gamepad_state gp_prev;
    odroid_input_gamepad_read(&gp_prev);

    /* Spin up the core-1 display/audio consumer. */
    s_free_q    = xQueueCreate(NB, sizeof(int));
    s_disp_q    = xQueueCreate(NB, sizeof(int));
    s_disp_done = xSemaphoreCreateBinary();
    if (!s_free_q || !s_disp_q || !s_disp_done) { ESP_LOGE(TAG, "pipeline alloc failed"); goto cleanup_loaded; }
    for (int i = 0; i < NB; i++) { int idx = i; xQueueSend(s_free_q, &idx, 0); }
    if (xTaskCreatePinnedToCore(vgba_display_task, "vgba_disp", 4096, NULL, 5, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "display task create failed"); goto cleanup_loaded;
    }

    int frame = 0;
    int64_t fps_t = esp_timer_get_time();
    int64_t acc_cpu = 0, acc_wait = 0; uint64_t acc_cyc = 0;
#if VGBA_FRAME_LIMIT
    int64_t frame_deadline = esp_timer_get_time();
#endif

    while (!s_quit) {
        int idx;
        int64_t t0 = esp_timer_get_time();
        xQueueReceive(s_free_q, &idx, portMAX_DELAY);    /* wait for a free buffer */
        int64_t t1 = esp_timer_get_time();
        acc_wait += t1 - t0;

        s_cur = idx;
        s_aud_n[idx] = 0;
        uint32_t _cyc0 = esp_cpu_get_cycle_count();
        retro_run();                                     /* emulate; video+audio cbs fill buffer idx */
        acc_cyc += (uint32_t)(esp_cpu_get_cycle_count() - _cyc0);
        acc_cpu += esp_timer_get_time() - t1;
#if VGBA_FPS_OVERLAY
        draw_fps_overlay(s_fb[idx], s_cur_fps);           /* live FPS in the corner */
#endif

        xQueueSend(s_disp_q, &idx, portMAX_DELAY);       /* hand to core 1 (blit + audio) */

        odroid_gamepad_state gp;
        odroid_input_gamepad_read(&gp);
        if (gp.values[ODROID_INPUT_MENU] && !gp_prev.values[ODROID_INPUT_MENU]) { s_quit = true; }
        /* VOL (on-screen) cycles volume: mute -> 10 -> 30 -> 50 -> 100 -> mute
         * (odroid_audio volume_pct_table), applied to the ES8388 + persisted. */
        if (gp.values[ODROID_INPUT_VOLUME] && !gp_prev.values[ODROID_INPUT_VOLUME]) {
            s_vol_req = 1;   /* applied on core1 (display task) to avoid a codec race */
        }
        /* SELECT+R cycles frameskip 0->1->2: skipped frames don't render, so the
         * ~5 ms PPU cost is saved on them -> emulation runs closer to full speed
         * (at the cost of choppier visuals). Emulation itself runs every frame. */
        if (gp.values[ODROID_INPUT_SELECT] && gp.values[ODROID_INPUT_R] && !gp_prev.values[ODROID_INPUT_R]) {
            s_frameskip = (s_frameskip + 1) % 3;
            SetFrameskip(s_frameskip & 0x0F);
            ESP_LOGW(TAG, "frameskip = %d", s_frameskip);
        }
        gp_prev = gp;

#if VGBA_FRAME_LIMIT
        /* Frame limiter — pace to native ~59.73 fps so JIT overspeed can't outrun
         * the 48kHz audio sink (GB APU blip overflow). Yield a full tick while there
         * is >1 tick of slack, then spin the sub-tick remainder for precision. */
        frame_deadline += VGBA_FRAME_US;
        int64_t pnow;
        while ((pnow = esp_timer_get_time()) < frame_deadline) {
            if ((frame_deadline - pnow) / 1000 >= (int64_t)portTICK_PERIOD_MS + 1)
                vTaskDelay(1);
            /* else spin */
        }
        if (esp_timer_get_time() - frame_deadline > VGBA_FRAME_US * 4)
            frame_deadline = esp_timer_get_time();   /* fell far behind (heavy scene) → resync */
#endif

        if (++frame >= 60) {
            int64_t now = esp_timer_get_time();
            s_cur_fps = (int)(frame * 1000000LL / (now - fps_t));
            s_cur_cpu = (int)(acc_cpu / frame / 1000);
            extern unsigned g_vgba_blocks, g_vgba_jit_instrs, g_vgba_arm_instrs;
            static unsigned s_arm_instrs_prev = 0;
            unsigned ji = g_vgba_jit_instrs - s_jit_instrs_prev; s_jit_instrs_prev = g_vgba_jit_instrs;
            unsigned ai = g_vgba_arm_instrs - s_arm_instrs_prev; s_arm_instrs_prev = g_vgba_arm_instrs;
            ESP_LOGI(TAG, "FPS=%d  CPU=%dms WAIT=%lldms | JIT blocks=%u instrs/frame=%u ARM-jit/frame=%u",
                     s_cur_fps, s_cur_cpu, acc_wait / frame / 1000, g_vgba_blocks, ji / frame, ai / frame);
            { extern unsigned g_arm_region[16];
              ESP_LOGW(TAG, "ARM-region/frame: EWRAM(02)=%u IWRAM(03)=%u ROM(08)=%u ROM(09)=%u other=%u",
                       g_arm_region[2]/frame, g_arm_region[3]/frame, g_arm_region[8]/frame, g_arm_region[9]/frame,
                       (g_arm_region[0]+g_arm_region[1]+g_arm_region[4]+g_arm_region[5]+g_arm_region[6]+g_arm_region[7])/frame);
              for (int i = 0; i < 16; i++) g_arm_region[i] = 0; }
            { extern unsigned g_iprof_arm, g_iprof_thumb, g_iprof[6];
              unsigned long n = (unsigned long)g_iprof_arm + g_iprof_thumb;
              if (n > 1) {
                  unsigned long ipf = n / frame;
                  unsigned long cpf = (unsigned long)(acc_cyc / frame);
                  ESP_LOGW(TAG, "CPI: instr/f=%lu cyc/f=%lu CPI=%lu | LDST=%lu%% BLK=%lu%% ALU=%lu%%",
                           ipf, cpf, ipf ? cpf / ipf : 0,
                           100UL * g_iprof[2] / n, 100UL * g_iprof[3] / n, 100UL * g_iprof[0] / n);
              }
              g_iprof_arm = 0;
              g_iprof_thumb = 0;
              for (int i = 0; i < 6; i++) g_iprof[i] = 0;
            }
            acc_cyc = 0;
            /* M6.5: in profile mode (g_vgba_jit==2) report where the hot THUMB runs
             * from + how translatable it is — decides ROM-only vs needing IWRAM+SMC. */
            extern int g_vgba_jit; extern unsigned g_vgba_prof_reg[4], g_vgba_prof_xlat[4];
            if (g_vgba_jit == 2) {
                unsigned *R = g_vgba_prof_reg, *X = g_vgba_prof_xlat;
                #define PCT(x, t) ((t) ? (unsigned)((uint64_t)(x) * 100 / (t)) : 0u)
                ESP_LOGW(TAG, "PROF/frame EWRAM=%u(xlat %u%%) IWRAM=%u(xlat %u%%) ROM=%u(xlat %u%%) other=%u",
                         R[0] / frame, PCT(X[0], R[0]), R[1] / frame, PCT(X[1], R[1]),
                         R[2] / frame, PCT(X[2], R[2]), R[3] / frame);
                #undef PCT
                R[0] = R[1] = R[2] = R[3] = 0; X[0] = X[1] = X[2] = X[3] = 0;
            }
            frame = 0; fps_t = now; acc_cpu = 0; acc_wait = 0;
        }
    }

    /* Drain the pipeline: sentinel stops the display task; wait for it to exit. */
    { int stop = -1; xQueueSend(s_disp_q, &stop, portMAX_DELAY); }
    xSemaphoreTake(s_disp_done, portMAX_DELAY);

    /* Persist save RAM to .sav. */
    {
        char save[512]; sav_path(rom_path, save, sizeof(save));
        void  *sram = retro_get_memory_data(RETRO_MEMORY_SAVE_RAM);
        size_t scap = retro_get_memory_size(RETRO_MEMORY_SAVE_RAM);
        if (sram && scap) {
            FILE *sf = fopen(save, "wb");
            if (sf) { fwrite(sram, 1, scap, sf); fclose(sf); ESP_LOGI(TAG, "saved: %u bytes", (unsigned)scap); }
        }
    }

cleanup_loaded:   /* reached by the normal exit AND by late post-load failures (pipeline/
                   * display-task) — undo audio/input/portrait + unload the game. Does NOT
                   * drain the display pipeline (the late paths never started the task). */
    odroid_audio_terminate();
    odroid_input_xy_menu_disable = false;
    odroid_vpad_disable();
    display_set_emu_portrait(false);

    retro_unload_game();
cleanup_core:
    retro_deinit();
cleanup:
    if (s_disp_q)    { vQueueDelete(s_disp_q);    s_disp_q = NULL; }
    if (s_free_q)    { vQueueDelete(s_free_q);    s_free_q = NULL; }
    if (s_disp_done) { vSemaphoreDelete(s_disp_done); s_disp_done = NULL; }
    for (int i = 0; i < NB; i++) {
        if (s_fb[i])      { heap_caps_free(s_fb[i]);      s_fb[i] = NULL; }
        if (s_aud_acc[i]) { heap_caps_free(s_aud_acc[i]); s_aud_acc[i] = NULL; }
    }
    if (s_aud_rs)  { heap_caps_free(s_aud_rs);  s_aud_rs = NULL; }
    if (s_rom)     { heap_caps_free(s_rom);     s_rom = NULL; }
}

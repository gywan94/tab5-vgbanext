#include "odroid_vpad.h"
#include "vpad_font8.h"
#include "bsp/m5stack_tab5.h"
#include "esp_lcd_panel_ops.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "vpad";

/* Panel geometry: 720x1280 portrait. The game is drawn upright in the top
 * region (panel y in [0, PAD_Y0)); these buttons fill the bottom region
 * (panel y in [PAD_Y0, 1280)). */
#define PANEL_W   720
#define PANEL_H   1280
#define PAD_Y0    640
#define PAD_H     (PANEL_H - PAD_Y0)   /* 640 */

/* Button hit rectangles in PANEL coordinates (x0,y0,x1,y1, button index).
 * D-pad zones overlap so a single finger near a corner triggers a diagonal.
 * Layout: L/R shoulders top, D-pad left, A/B/X/Y diamond right,
 * SELECT/START/MENU/VOL along the bottom. */
typedef struct { int x0, y0, x1, y1, btn; } vbtn_t;
static const vbtn_t VBTNS[] = {
    /* Shoulder buttons */
    { 40, 655, 260,  725, ODROID_INPUT_L},
    {460, 655, 680,  725, ODROID_INPUT_R},
    /* D-pad (left) */
    {105, 745, 225,  870, ODROID_INPUT_UP},
    {105, 870, 225,  995, ODROID_INPUT_DOWN},
    { 45, 800, 165,  940, ODROID_INPUT_LEFT},
    {165, 800, 285,  940, ODROID_INPUT_RIGHT},
    /* Face buttons (right) — SNES diamond: X top, Y left, A right, B bottom */
    {508, 733, 612,  837, ODROID_INPUT_X},
    {423, 818, 527,  922, ODROID_INPUT_Y},
    {593, 818, 697,  922, ODROID_INPUT_A},
    {508, 903, 612, 1007, ODROID_INPUT_B},
    /* System buttons (bottom row) */
    { 57,1095, 197, 1165, ODROID_INPUT_SELECT},
    {212,1095, 352, 1165, ODROID_INPUT_START},
    {367,1095, 507, 1165, ODROID_INPUT_MENU},
    {522,1095, 662, 1165, ODROID_INPUT_VOLUME},
};
#define NBTN (sizeof(VBTNS) / sizeof(VBTNS[0]))

static uint16_t *s_buf = NULL;   /* PAD: 720 x PAD_H RGB565 */
static bool s_active = false;

/* Palette (RGB565) — charcoal base + single cyan accent, matching the browser. */
#define C_BG     0x0861   /* charcoal background        */
#define C_DPAD   0x39C7   /* d-pad slate                */
#define C_A      0xE145   /* red    (GBA A)             */
#define C_B      0x2DEB   /* green  (GBA B)             */
#define C_X      0x33BF   /* blue                       */
#define C_Y      0xFCE6   /* amber                      */
#define C_SHLDR  0x2945   /* shoulder slate             */
#define C_SYS    0x2104   /* system-button base         */
#define C_ACCENT 0x05FF   /* cyan accent                */
#define C_TXT    0xFFFF
#define C_TXTDIM 0x8C71

/* Lighten / darken an RGB565 colour for bevel edges. */
static inline uint16_t lighten(uint16_t c)
{
    int r = (c >> 11) & 31, g = (c >> 5) & 63, b = c & 31;
    r = r + 7 > 31 ? 31 : r + 7; g = g + 14 > 63 ? 63 : g + 14; b = b + 7 > 31 ? 31 : b + 7;
    return (uint16_t)((r << 11) | (g << 5) | b);
}
static inline uint16_t darken(uint16_t c)
{
    int r = (c >> 11) & 31, g = (c >> 5) & 63, b = c & 31;
    r = r > 7 ? r - 7 : 0; g = g > 14 ? g - 14 : 0; b = b > 7 ? b - 7 : 0;
    return (uint16_t)((r << 11) | (g << 5) | b);
}

static inline void px(int lx, int ly, uint16_t c)
{
    if (lx < 0 || lx >= PANEL_W || ly < 0 || ly >= PAD_H) return;
    s_buf[ly * PANEL_W + lx] = c;
}

static void fill_rect(int lx, int ly, int w, int h, uint16_t c)
{
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            px(lx + x, ly + y, c);
}

static void fill_circle(int cx, int cy, int r, uint16_t c)
{
    for (int y = -r; y <= r; y++)
        for (int x = -r; x <= r; x++)
            if (x * x + y * y <= r * r) px(cx + x, cy + y, c);
}

/* Rounded-corner filled rect (corner radius r). */
static void fill_rrect(int lx, int ly, int w, int h, int r, uint16_t c)
{
    if (r * 2 > w) r = w / 2;
    if (r * 2 > h) r = h / 2;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int dx = -1, dy = -1;
            if (x < r && y < r)             { dx = r - 1 - x; dy = r - 1 - y; }
            else if (x >= w - r && y < r)   { dx = x - (w - r); dy = r - 1 - y; }
            else if (x < r && y >= h - r)   { dx = r - 1 - x; dy = y - (h - r); }
            else if (x >= w - r && y >= h - r){ dx = x - (w - r); dy = y - (h - r); }
            if (dx >= 0 && dx * dx + dy * dy > r * r) continue;
            px(lx + x, ly + y, c);
        }
    }
}

/* Beveled rounded rect: base fill + lighter top/left edge + darker bottom/right. */
static void bevel_rrect(int lx, int ly, int w, int h, int r, uint16_t base)
{
    uint16_t hi = lighten(base), lo = darken(base);
    fill_rrect(lx, ly, w, h, r, base);
    fill_rect(lx + r, ly, w - 2 * r, 3, hi);              /* top    */
    fill_rect(lx, ly + r, 3, h - 2 * r, hi);              /* left   */
    fill_rect(lx + r, ly + h - 3, w - 2 * r, 3, lo);      /* bottom */
    fill_rect(lx + w - 3, ly + r, 3, h - 2 * r, lo);      /* right  */
}

/* Beveled circle: base + ~3px rim (lighter upper-left, darker lower-right). */
static void bevel_circle(int cx, int cy, int r, uint16_t base)
{
    uint16_t hi = lighten(base), lo = darken(base);
    fill_circle(cx, cy, r, base);
    int rin = (r - 3) * (r - 3), rout = r * r;
    for (int y = -r; y <= r; y++)
        for (int x = -r; x <= r; x++) {
            int d = x * x + y * y;
            if (d <= rout && d > rin) px(cx + x, cy + y, (x + y < 0) ? hi : lo);
        }
}

/* Small solid direction triangles for the d-pad. */
static void tri_up(int cx, int cy, int s, uint16_t c)   { for (int i = 0; i < s; i++) fill_rect(cx - i, cy + i, 2 * i + 1, 1, c); }
static void tri_down(int cx, int cy, int s, uint16_t c) { for (int i = 0; i < s; i++) fill_rect(cx - i, cy - i, 2 * i + 1, 1, c); }
static void tri_left(int cx, int cy, int s, uint16_t c) { for (int i = 0; i < s; i++) fill_rect(cx + i, cy - i, 1, 2 * i + 1, c); }
static void tri_right(int cx, int cy, int s, uint16_t c){ for (int i = 0; i < s; i++) fill_rect(cx - i, cy - i, 1, 2 * i + 1, c); }

/* Draw one ASCII glyph scaled by `s` at local (lx,ly). */
static void blit_char(int lx, int ly, char ch, int s, uint16_t c)
{
    if (ch < 0x20 || ch > 0x7e) return;
    const uint8_t *g = vpad_font8[ch - 0x20];
    for (int row = 0; row < 16; row++)
        for (int col = 0; col < 8; col++)
            if (g[row] & (0x80 >> col))
                fill_rect(lx + col * s, ly + row * s, s, s, c);
}

/* Draw a string centered horizontally on cx (local coords), baseline top = ly. */
static void blit_text_centered(int cx, int ly, const char *str, int s, uint16_t c)
{
    int w = (int)strlen(str) * 8 * s;
    int x = cx - w / 2;
    for (const char *p = str; *p; p++) {
        blit_char(x, ly, *p, s, c);
        x += 8 * s;
    }
}

static inline int L(int panel_y) { return panel_y - PAD_Y0; }  /* panel->local y */

/* Draw a labelled face button (beveled circle + centered char). */
static void draw_face(int cx, int cy, int r, uint16_t fill, char label)
{
    fill_circle(cx, cy, r + 3, darken(C_BG));   /* socket shadow ring */
    bevel_circle(cx, cy, r, fill);
    blit_char(cx - 12, cy - 24, label, 3, C_TXT);
}

/* Beveled, rounded system/shoulder button with a centered label. */
static void draw_btn(int x0, int y0, int w, int h, uint16_t base,
                     const char *label, int tsize, uint16_t tcol)
{
    bevel_rrect(x0, L(y0), w, h, 12, base);
    blit_text_centered(x0 + w / 2, L(y0) + (h - 16 * tsize) / 2, label, tsize, tcol);
}

void odroid_vpad_draw(void)
{
    if (!s_buf) {
        s_buf = heap_caps_aligned_calloc(64, 1, (size_t)PANEL_W * PAD_H * 2,
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        if (!s_buf) {
            ESP_LOGE(TAG, "vpad buffer alloc failed");
            return;
        }
    }

    /* background + a thin accent divider under the game area */
    fill_rect(0, 0, PANEL_W, PAD_H, C_BG);
    fill_rect(0, 0, PANEL_W, 3, C_ACCENT);

    /* L / R shoulder buttons (rounded, beveled) */
    draw_btn(40,  655, 220, 70, C_SHLDR, "L", 3, C_TXT);
    draw_btn(460, 655, 220, 70, C_SHLDR, "R", 3, C_TXT);

    /* D-pad: beveled cross + center hub + cyan direction arrows */
    bevel_rrect(105, L(745), 120, 250, 16, C_DPAD);   /* vertical bar   */
    bevel_rrect(45,  L(800), 240, 140, 16, C_DPAD);   /* horizontal bar */
    fill_circle(165, L(870), 30, darken(C_DPAD));     /* center hub      */
    tri_up   (165, L(762), 16, C_ACCENT);
    tri_down (165, L(978), 16, C_ACCENT);
    tri_left (62,  L(870), 16, C_ACCENT);
    tri_right(268, L(870), 16, C_ACCENT);

    /* Face buttons — SNES diamond (X top, Y left, A right, B bottom) */
    draw_face(560, L(785), 52, C_X, 'X');
    draw_face(475, L(870), 52, C_Y, 'Y');
    draw_face(645, L(870), 52, C_A, 'A');
    draw_face(560, L(955), 52, C_B, 'B');

    /* SELECT / START / MENU / VOL (MENU highlighted — it exits the game) */
    draw_btn( 57, 1095, 140, 70, C_SYS, "SEL",   2, C_TXTDIM);
    draw_btn(212, 1095, 140, 70, C_SYS, "START", 1, C_TXT);
    draw_btn(367, 1095, 140, 70, C_SYS, "MENU",  1, C_ACCENT);
    draw_btn(522, 1095, 140, 70, C_SYS, "VOL",   2, C_TXTDIM);

    esp_lcd_panel_handle_t panel = bsp_get_panel_handle();
    if (panel) {
        esp_cache_msync(s_buf, (size_t)PANEL_W * PAD_H * 2, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        esp_lcd_panel_draw_bitmap(panel, 0, PAD_Y0, PANEL_W, PANEL_H, s_buf);
    }
    s_active = true;
    ESP_LOGI(TAG, "virtual gamepad drawn (SNES layout, bottom half)");
}

void odroid_vpad_disable(void)
{
    s_active = false;
}

bool odroid_vpad_is_active(void)
{
    return s_active;
}

void odroid_vpad_poll(odroid_gamepad_state *state)
{
    if (!s_active) return;

    /* The game loop polls input at ~90 Hz, but the ST7123 touch controller
     * doesn't like being read over I2C that fast. Rate-limit the actual touch
     * read to ~60 Hz and reuse the cached button state in between. */
    static int64_t last_us = 0;
    static int cached[ODROID_INPUT_MAX] = {0};

    int64_t now = esp_timer_get_time();
    if (now - last_us >= 16000) {
        last_us = now;
        memset(cached, 0, sizeof(cached));

        uint16_t xs[8], ys[8];
        int n = bsp_touch_read_points(xs, ys, 8);
        for (int i = 0; i < n; i++) {
            for (size_t b = 0; b < NBTN; b++) {
                if (xs[i] >= VBTNS[b].x0 && xs[i] < VBTNS[b].x1 &&
                    ys[i] >= VBTNS[b].y0 && ys[i] < VBTNS[b].y1) {
                    cached[VBTNS[b].btn] = 1;
                }
            }
        }
    }

    for (int i = 0; i < ODROID_INPUT_MAX; i++) {
        state->values[i] |= cached[i];
    }
}

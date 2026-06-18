#include "file_browser.h"
#include "odroid_display.h"
#include "bsp/m5stack_tab5.h"
#include "esp_lcd_panel_ops.h"
#include "cjk_font16.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <dirent.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>

static const char *TAG = "file_browser";

#define MAX_FILES       256
#define MAX_PATH        512
#define ROW_HEIGHT      26
#define TITLE_H         30
#define SCREEN_W        320
#define SCREEN_H        240

/* Colors (RGB565) — tuned to match the UserDemo SD browser look. */
#define COLOR_BG        0x10A2   /* near-black list background       */
#define COLOR_TITLE_BG  0x2945   /* dark gray header                 */
#define COLOR_TITLE_FG  0xFFFF
#define COLOR_ROW_BG    0x10A2
#define COLOR_ROW_ALT   0x18E3   /* zebra striping for readability   */
#define COLOR_ROW_SEL   0x4208   /* pressed/selected row highlight   */
#define COLOR_ROW_FG    0xFFFF
#define COLOR_DIR_FG    0xFDE3   /* folder = amber (demo 0xFDBE1A)   */
#define COLOR_FILE_FG   0x469F   /* file   = cyan  (demo 0x43D2FF)   */
#define COLOR_SCROLLBAR 0x52AA
#define COLOR_SCROLLTRK 0x18E3

typedef struct {
    char name[128];
    char full_path[MAX_PATH];
    int  name_len;
    bool is_dir;
} file_entry_t;

static file_entry_t s_files[MAX_FILES];
static int s_file_count = 0;
static int s_scroll_offset = 0;
static int s_selected = 0;
static char s_current_path[MAX_PATH];
static bool s_running = false;
static uint16_t *s_fb = NULL;

#include <stdint.h>
static const uint8_t font_8x16[95][16] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x18,0x3C,0x3C,0x3C,0x18,0x18,0x18,0x00,0x18,0x18,0x00,0x00,0x00,0x00},
    {0x00,0x66,0x66,0x66,0x24,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x6C,0x6C,0xFE,0x6C,0x6C,0x6C,0xFE,0x6C,0x6C,0x00,0x00,0x00,0x00},
    {0x18,0x18,0x7C,0xC6,0xC2,0xC0,0x7C,0x06,0x86,0xC6,0x7C,0x18,0x18,0x00,0x00,0x00},
    {0x00,0x00,0x00,0xC2,0xC6,0x0C,0x18,0x30,0x60,0xC6,0x86,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x38,0x6C,0x6C,0x38,0x76,0xDC,0xCC,0xCC,0xCC,0x76,0x00,0x00,0x00,0x00},
    {0x00,0x30,0x30,0x30,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x0C,0x18,0x30,0x30,0x30,0x30,0x30,0x30,0x18,0x0C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x30,0x18,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x18,0x30,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x18,0x30,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x02,0x06,0x0C,0x18,0x30,0x60,0xC0,0x80,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x7C,0xC6,0xC6,0xCE,0xD6,0xD6,0xE6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x18,0x38,0x78,0x18,0x18,0x18,0x18,0x18,0x18,0x7E,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x7C,0xC6,0x06,0x0C,0x18,0x30,0x60,0xC0,0xC6,0xFE,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x7C,0xC6,0x06,0x06,0x3C,0x06,0x06,0x06,0xC6,0x7C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x0C,0x1C,0x3C,0x6C,0xCC,0xFE,0x0C,0x0C,0x0C,0x1E,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xFE,0xC0,0xC0,0xC0,0xFC,0x06,0x06,0x06,0xC6,0x7C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x38,0x60,0xC0,0xC0,0xFC,0xC6,0xC6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xFE,0xC6,0x06,0x06,0x0C,0x18,0x30,0x30,0x30,0x30,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x7C,0xC6,0xC6,0xC6,0x7C,0xC6,0xC6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x7C,0xC6,0xC6,0xC6,0x7E,0x06,0x06,0x06,0x0C,0x78,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x18,0x18,0x30,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x06,0x0C,0x18,0x30,0x60,0x30,0x18,0x0C,0x06,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0xFE,0x00,0x00,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x60,0x30,0x18,0x0C,0x06,0x0C,0x18,0x30,0x60,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x7C,0xC6,0xC6,0x0C,0x18,0x18,0x18,0x00,0x18,0x18,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x7C,0xC6,0xC6,0xDE,0xDE,0xDE,0xDC,0xC0,0x7C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x10,0x38,0x6C,0xC6,0xC6,0xFE,0xC6,0xC6,0xC6,0xC6,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xFC,0x66,0x66,0x66,0x7C,0x66,0x66,0x66,0x66,0xFC,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x3C,0x66,0xC2,0xC0,0xC0,0xC0,0xC0,0xC2,0x66,0x3C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xF8,0x6C,0x66,0x66,0x66,0x66,0x66,0x66,0x6C,0xF8,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xFE,0x66,0x62,0x68,0x78,0x68,0x60,0x62,0x66,0xFE,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xFE,0x66,0x62,0x68,0x78,0x68,0x60,0x60,0x60,0xF0,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x3C,0x66,0xC2,0xC0,0xC0,0xDE,0xC6,0xC6,0x66,0x3A,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xC6,0xC6,0xC6,0xC6,0xFE,0xC6,0xC6,0xC6,0xC6,0xC6,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x3C,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0xCC,0xCC,0xCC,0x78,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xE6,0x66,0x6C,0x6C,0x78,0x78,0x6C,0x66,0x66,0xE6,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xF0,0x60,0x60,0x60,0x60,0x60,0x60,0x62,0x66,0xFE,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xC6,0xEE,0xFE,0xFE,0xD6,0xC6,0xC6,0xC6,0xC6,0xC6,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xC6,0xE6,0xF6,0xFE,0xDE,0xCE,0xC6,0xC6,0xC6,0xC6,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x38,0x6C,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0x6C,0x38,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xFC,0x66,0x66,0x66,0x7C,0x60,0x60,0x60,0x60,0xF0,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x7C,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0xD6,0xDE,0x7C,0x0C,0x0E,0x00,0x00},
    {0x00,0x00,0xFC,0x66,0x66,0x66,0x7C,0x6C,0x66,0x66,0x66,0xE6,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x7C,0xC6,0xC6,0xC0,0x70,0x1C,0x06,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x7E,0x7E,0x5A,0x18,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0x6C,0x38,0x10,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xC6,0xC6,0xC6,0xC6,0xC6,0xD6,0xD6,0xFE,0x6C,0x6C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xC6,0xC6,0x6C,0x6C,0x38,0x38,0x6C,0x6C,0xC6,0xC6,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x66,0x66,0x66,0x66,0x3C,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xFE,0xC6,0x86,0x0C,0x18,0x30,0x60,0xC2,0xC6,0xFE,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x3C,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x3C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x80,0xC0,0x60,0x30,0x18,0x0C,0x06,0x02,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00,0x00,0x00,0x00},
    {0x10,0x38,0x6C,0xC6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00},
    {0x30,0x30,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x78,0x0C,0x7C,0xCC,0xCC,0xCC,0x76,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xE0,0x60,0x60,0x78,0x6C,0x66,0x66,0x66,0x66,0x7C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x7C,0xC6,0xC0,0xC0,0xC0,0xC6,0x7C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x1C,0x0C,0x0C,0x3C,0x6C,0xCC,0xCC,0xCC,0xCC,0x76,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x7C,0xC6,0xFE,0xC0,0xC0,0xC6,0x7C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x38,0x6C,0x64,0x60,0xF0,0x60,0x60,0x60,0x60,0xF0,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x76,0xCC,0xCC,0xCC,0xCC,0xCC,0x7C,0x0C,0xCC,0x78,0x00},
    {0x00,0x00,0xE0,0x60,0x60,0x6C,0x76,0x66,0x66,0x66,0x66,0xE6,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x18,0x18,0x00,0x38,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x0C,0x0C,0x00,0x1C,0x0C,0x0C,0x0C,0x0C,0x0C,0xCC,0xCC,0x78,0x00,0x00},
    {0x00,0x00,0xE0,0x60,0x60,0x66,0x6C,0x78,0x78,0x6C,0x66,0xE6,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x38,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0xEC,0xFE,0xD6,0xD6,0xD6,0xD6,0xC6,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0xDC,0x66,0x66,0x66,0x66,0x66,0x66,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x7C,0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0xDC,0x66,0x66,0x66,0x66,0x66,0x7C,0x60,0x60,0xF0,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x76,0xCC,0xCC,0xCC,0xCC,0xCC,0x7C,0x0C,0x0C,0x1E,0x00},
    {0x00,0x00,0x00,0x00,0x00,0xDC,0x76,0x66,0x60,0x60,0x60,0xF0,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x7C,0xC6,0x60,0x38,0x0C,0xC6,0x7C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x10,0x30,0x30,0xFC,0x30,0x30,0x30,0x30,0x36,0x1C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0x76,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0xC6,0xC6,0xC6,0xC6,0x6C,0x38,0x10,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0xC6,0xC6,0xD6,0xD6,0xD6,0xFE,0x6C,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0xC6,0x6C,0x38,0x38,0x38,0x6C,0xC6,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0x7E,0x06,0x0C,0xF8,0x00},
    {0x00,0x00,0x00,0x00,0x00,0xFE,0xCC,0x18,0x30,0x60,0xC6,0xFE,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x0E,0x18,0x18,0x18,0x70,0x18,0x18,0x18,0x18,0x0E,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x70,0x18,0x18,0x18,0x0E,0x18,0x18,0x18,0x18,0x70,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x76,0xDC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
};

static void draw_char(int x, int y, char c, uint16_t fg, uint16_t bg)
{
    if (c < 32 || c > 126) c = ' ';
    int idx = c - 32;
    uint16_t *fb = s_fb;
    for (int row = 0; row < 16; row++) {
        uint8_t bits = font_8x16[idx][row];
        for (int col = 0; col < 8; col++) {
            int px = x + col;
            int py = y + row;
            if (px < 0 || px >= SCREEN_W || py < 0 || py >= SCREEN_H) continue;
            fb[py * SCREEN_W + px] = (bits & (0x80 >> col)) ? fg : bg;
        }
    }
}

/* Draw a 16x16 CJK glyph (codepoint cp) at (x,y). Falls back to a hollow box
 * if the glyph isn't in the GB2312 font. */
static void draw_cjk(int x, int y, uint32_t cp, uint16_t fg, uint16_t bg)
{
    const uint8_t *g = cjk_glyph(cp);
    for (int row = 0; row < 16; row++) {
        uint16_t bits = g ? ((g[row * 2] << 8) | g[row * 2 + 1]) : 0;
        for (int col = 0; col < 16; col++) {
            int px = x + col, py = y + row;
            if (px < 0 || px >= SCREEN_W || py < 0 || py >= SCREEN_H) continue;
            s_fb[py * SCREEN_W + px] = (bits & (0x8000 >> col)) ? fg : bg;
        }
    }
}

/* Decode one UTF-8 codepoint; returns pointer past the consumed bytes. */
static const char *utf8_next(const char *s, uint32_t *cp)
{
    unsigned char c = (unsigned char)s[0];
    if (c < 0x80) { *cp = c; return s + 1; }
    if ((c >> 5) == 0x6 && (s[1] & 0xc0) == 0x80) {
        *cp = ((c & 0x1f) << 6) | (s[1] & 0x3f);
        return s + 2;
    }
    if ((c >> 4) == 0xe && (s[1] & 0xc0) == 0x80 && (s[2] & 0xc0) == 0x80) {
        *cp = ((c & 0x0f) << 12) | ((s[1] & 0x3f) << 6) | (s[2] & 0x3f);
        return s + 3;
    }
    if ((c >> 3) == 0x1e && (s[1] & 0xc0) == 0x80 && (s[2] & 0xc0) == 0x80 && (s[3] & 0xc0) == 0x80) {
        *cp = ((c & 0x07) << 18) | ((s[1] & 0x3f) << 12) | ((s[2] & 0x3f) << 6) | (s[3] & 0x3f);
        return s + 4;
    }
    *cp = c;            /* invalid byte: render as-is, advance 1 */
    return s + 1;
}

/* UTF-8 aware string draw: ASCII glyphs are 8px wide, CJK glyphs 16px.
 * max_chars limits the number of glyphs drawn (0 = no limit). */
static void draw_string(int x, int y, const char *str, uint16_t fg, uint16_t bg, int max_chars)
{
    int cx = x;
    int count = 0;
    while (*str && (max_chars == 0 || count < max_chars)) {
        uint32_t cp;
        str = utf8_next(str, &cp);
        if (cp < 0x80) {
            draw_char(cx, y, (char)cp, fg, bg);
            cx += 8;
        } else {
            draw_cjk(cx, y, cp, fg, bg);
            cx += 16;
        }
        count++;
        if (cx >= SCREEN_W) break;
    }
}

/* Display width in pixels (ASCII=8, CJK=16). */
static int str_disp_width(const char *str)
{
    int w = 0;
    uint32_t cp;
    while (*str) { str = utf8_next(str, &cp); w += (cp < 0x80) ? 8 : 16; }
    return w;
}

static void draw_string_centered(int y, const char *str, uint16_t fg, uint16_t bg)
{
    int x = (SCREEN_W - str_disp_width(str)) / 2;
    if (x < 0) x = 0;
    draw_string(x, y, str, fg, bg, 0);
}

static void fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > SCREEN_W) w = SCREEN_W - x;
    if (y + h > SCREEN_H) h = SCREEN_H - y;
    if (w <= 0 || h <= 0) return;
    for (int row = 0; row < h; row++) {
        for (int col = 0; col < w; col++) {
            s_fb[(y + row) * SCREEN_W + (x + col)] = color;
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  Native 720×1280 renderer (Phase 1 retro theme)
 *  Draws directly into a full-panel buffer and blits via the LCD panel (no PPA
 *  upscale → crisp), portrait-native so touch maps 1:1 to panel coords.
 * ════════════════════════════════════════════════════════════════════════ */
#define NW 720
#define NH 1280
#define HEADER_H   150
#define FOOTER_H   88
#define ROWH       66
#define LIST_Y     HEADER_H
#define LIST_H     (NH - HEADER_H - FOOTER_H)
#define VIS_ROWS   (LIST_H / ROWH)

/* Retro indigo/cyan theme (RGB565). */
#define T_BG       0x0861   /* near-black indigo            */
#define T_HEADER   0x180D   /* deep indigo header bar       */
#define T_ACCENT   0x05FF   /* cyan accent                  */
#define T_ACCENT2  0xFD60   /* amber (folders)              */
#define T_SEL      0x2A6B   /* selected row fill            */
#define T_ROW      0x0861
#define T_ROW_ALT  0x1082   /* subtle zebra                 */
#define T_TXT      0xFFFF
#define T_DIM      0x8C71   /* muted gray text              */
#define T_FOOTER   0x1082

static uint16_t *s_canvas = NULL;   /* 720×1280 RGB565, DMA-capable */

static inline void nfill(int x, int y, int w, int h, uint16_t c)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > NW) w = NW - x;
    if (y + h > NH) h = NH - y;
    if (w <= 0 || h <= 0 || !s_canvas) return;
    for (int r = 0; r < h; r++) {
        uint16_t *row = s_canvas + (size_t)(y + r) * NW + x;
        for (int col = 0; col < w; col++) row[col] = c;
    }
}

/* 8×16 ASCII glyph at integer scale s, transparent background. */
static void nchar(int x, int y, char c, int s, uint16_t fg)
{
    if (c < 32 || c > 126) c = ' ';
    const uint8_t *g = font_8x16[c - 32];
    for (int row = 0; row < 16; row++) {
        uint8_t bits = g[row];
        for (int col = 0; col < 8; col++) {
            if (!(bits & (0x80 >> col))) continue;
            nfill(x + col * s, y + row * s, s, s, fg);
        }
    }
}

/* 16×16 CJK glyph at scale s, transparent background. */
static void ncjk(int x, int y, uint32_t cp, int s, uint16_t fg)
{
    const uint8_t *g = cjk_glyph(cp);
    if (!g) { nfill(x, y, 16 * s, 16 * s, T_DIM); return; }
    for (int row = 0; row < 16; row++) {
        uint16_t bits = (g[row * 2] << 8) | g[row * 2 + 1];
        for (int col = 0; col < 16; col++) {
            if (!(bits & (0x8000 >> col))) continue;
            nfill(x + col * s, y + row * s, s, s, fg);
        }
    }
}

/* UTF-8 string at scale s, clipped to max_px width. Returns end x. */
static int nstr(int x, int y, const char *str, int s, uint16_t fg, int max_px)
{
    int x0 = x;
    while (*str) {
        uint32_t cp;
        str = utf8_next(str, &cp);
        int w = (cp < 0x80 ? 8 : 16) * s;
        if (max_px && (x - x0 + w) > max_px) break;
        if (cp < 0x80) nchar(x, y, (char)cp, s, fg);
        else           ncjk(x, y, cp, s, fg);
        x += w;
    }
    return x;
}

static int nstr_width(const char *str, int s)
{
    int w = 0; uint32_t cp;
    while (*str) { str = utf8_next(str, &cp); w += (cp < 0x80 ? 8 : 16) * s; }
    return w;
}

/* Folder / cartridge icons (drawn with rects), sized ~36px for the 66px row. */
static void nicon_folder(int x, int y, uint16_t c)
{
    nfill(x,      y + 4,  16, 5,  c);
    nfill(x,      y + 8,  38, 24, c);
    nfill(x + 3,  y + 12, 32, 18, T_HEADER);
    nfill(x + 3,  y + 12, 32, 18, c);
}
static void nicon_cart(int x, int y, uint16_t c)
{
    nfill(x + 4,  y + 2,  26, 32, c);     /* body  */
    nfill(x + 9,  y + 2,  16, 8,  T_BG);  /* label notch */
    nfill(x + 9,  y + 24, 16, 6,  T_BG);  /* contacts */
}

static void present_native(void)
{
    esp_cache_msync(s_canvas, (size_t)NW * NH * 2, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    esp_lcd_panel_handle_t panel = bsp_get_panel_handle();
    if (panel) esp_lcd_panel_draw_bitmap(panel, 0, 0, NW, NH, s_canvas);
}

static void draw_ui_native(void)
{
    if (!s_canvas) return;
    nfill(0, 0, NW, NH, T_BG);

    /* ── Header bar ── */
    nfill(0, 0, NW, HEADER_H, T_HEADER);
    nfill(0, HEADER_H - 4, NW, 4, T_ACCENT);                 /* accent underline */
    nstr(28, 26, "GBA", 5, T_ACCENT, 0);
    nstr(28 + 3 * 8 * 5 + 16, 42, "LIBRARY", 3, T_TXT, 0);

    /* breadcrumb (tail of the path) + item count */
    const char *shown = s_current_path;
    if (nstr_width(shown, 2) > NW - 120) {
        const char *slash = strrchr(s_current_path, '/');
        if (slash && slash != s_current_path) shown = slash + 1;
    }
    nstr(28, 104, shown, 2, T_DIM, NW - 140);
    char info[24];
    snprintf(info, sizeof(info), "%d items", s_file_count);
    nstr(NW - 28 - nstr_width(info, 2), 104, info, 2, T_ACCENT, 0);

    /* ── Rows ── */
    for (int i = 0; i < VIS_ROWS; i++) {
        int idx = s_scroll_offset + i;
        if (idx >= s_file_count) break;
        int y = LIST_Y + i * ROWH;
        file_entry_t *fe = &s_files[idx];
        bool sel = (idx == s_selected);

        uint16_t bg = sel ? T_SEL : ((i & 1) ? T_ROW_ALT : T_ROW);
        nfill(0, y, NW - 8, ROWH, bg);
        if (sel) nfill(0, y, 6, ROWH, T_ACCENT);            /* left accent stripe */

        int icon_y = y + (ROWH - 36) / 2;
        int name_y = y + (ROWH - 16 * 3) / 2;
        uint16_t nc;
        if (fe->is_dir) {
            if (strcmp(fe->name, "..") == 0) {
                nstr(30, name_y, "..", 3, T_ACCENT2, 0);
                nstr(96, name_y + 8, "up one level", 2, T_DIM, NW - 130);
                continue;
            }
            nicon_folder(28, icon_y, T_ACCENT2);
            nc = T_ACCENT2;
        } else {
            nicon_cart(28, icon_y, T_ACCENT);
            nc = T_TXT;
        }

        /* name without the .gba/.zip extension for a cleaner look */
        char nm[128];
        snprintf(nm, sizeof(nm), "%s", fe->name);
        if (!fe->is_dir) {
            char *dot = strrchr(nm, '.');
            if (dot && (strcasecmp(dot, ".gba") == 0 || strcasecmp(dot, ".zip") == 0)) *dot = '\0';
        }
        nstr(84, name_y, nm, 3, nc, NW - 120);
    }

    /* ── Scrollbar ── */
    if (s_file_count > VIS_ROWS) {
        nfill(NW - 8, LIST_Y, 8, LIST_H, T_ROW_ALT);
        int bar_h = (VIS_ROWS * LIST_H) / s_file_count;
        if (bar_h < 24) bar_h = 24;
        int span = LIST_H - bar_h;
        int bar_y = LIST_Y + (s_scroll_offset * span) / (s_file_count - VIS_ROWS);
        nfill(NW - 8, bar_y, 8, bar_h, T_ACCENT);
    }

    /* ── Footer hint bar ── */
    nfill(0, NH - FOOTER_H, NW, FOOTER_H, T_FOOTER);
    nfill(0, NH - FOOTER_H, NW, 3, T_ACCENT);
    nstr(28, NH - FOOTER_H + 30, "tap = open      swipe = scroll      header = back", 2, T_DIM, 0);

    present_native();
}

static int compare_entries(const void *a, const void *b)
{
    const file_entry_t *fa = (const file_entry_t *)a;
    const file_entry_t *fb = (const file_entry_t *)b;
    if (fa->is_dir && !fb->is_dir) return -1;
    if (!fa->is_dir && fb->is_dir) return 1;
    return strcasecmp(fa->name, fb->name);
}

static int scan_directory(const char *path)
{
    DIR *dir = opendir(path);
    if (!dir) {
        ESP_LOGE(TAG, "Cannot open dir: %s", path);
        return 0;
    }

    int count = 0;
    int seen = 0;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL && count < MAX_FILES) {
        if (strcmp(entry->d_name, ".") == 0) continue;

        char full[MAX_PATH];
        snprintf(full, sizeof(full), "%s/%s", path, entry->d_name);

        struct stat st;
        int strc = stat(full, &st);
        ESP_LOGD(TAG, "entry[%d]: '%s' stat=%d", seen++, entry->d_name, strc);
        if (strc != 0) continue;

        file_entry_t *fe = &s_files[count];
        snprintf(fe->name, sizeof(fe->name), "%s", entry->d_name);
        snprintf(fe->full_path, sizeof(fe->full_path), "%s", full);
        fe->name_len = strlen(fe->name);
        fe->is_dir = S_ISDIR(st.st_mode);

        if (fe->is_dir) {
            if (strcmp(entry->d_name, "..") == 0) {
                fe->name[0] = '.';
                fe->name[1] = '.';
                fe->name[2] = '\0';
                count++;
            } else {
                count++;
            }
        } else {
            const char *ext = strrchr(entry->d_name, '.');
            if (ext && (strcasecmp(ext, ".gba") == 0 || strcasecmp(ext, ".zip") == 0)) {
                count++;
            }
        }
    }
    closedir(dir);

    qsort(s_files, count, sizeof(file_entry_t), compare_entries);

    /* Prepend a synthetic ".." entry (unless at the SD root) so the user can
     * always tap to go up one directory. */
    if (strcmp(path, "/sd") != 0 && count < MAX_FILES) {
        memmove(&s_files[1], &s_files[0], (size_t)count * sizeof(file_entry_t));
        file_entry_t *up = &s_files[0];
        snprintf(up->name, sizeof(up->name), "..");
        up->name_len = 2;
        up->is_dir   = true;
        up->full_path[0] = '\0';
        count++;
    }

    ESP_LOGI(TAG, "scan: %d entries in %s", count, path);
    return count;
}

/* ── small 14×11 folder / file icons drawn with rects ─────────────── */
static void draw_folder_icon(int x, int y, uint16_t c, uint16_t bg)
{
    (void)bg;
    fill_rect(x,     y + 1, 6, 2,  c);   /* tab            */
    fill_rect(x,     y + 3, 14, 8, c);   /* body           */
}
static void draw_file_icon(int x, int y, uint16_t c, uint16_t bg)
{
    fill_rect(x + 1, y,     9, 11, c);   /* page           */
    fill_rect(x + 7, y,     3, 3,  bg);  /* folded corner  */
    fill_rect(x + 8, y + 1, 2, 2,  c);
}

static void draw_ui(void)
{
    /* Clear the whole 320×240 buffer first (shared with the emulator). */
    fill_rect(0, 0, SCREEN_W, SCREEN_H, COLOR_BG);

    int visible_rows = (SCREEN_H - TITLE_H) / ROW_HEIGHT;
    int list_h       = visible_rows * ROW_HEIGHT;

    /* ── Header: current path (right-aligned to last segment) + item count ── */
    fill_rect(0, 0, SCREEN_W, TITLE_H, COLOR_TITLE_BG);
    const char *shown = s_current_path;
    if (str_disp_width(shown) > (SCREEN_W - 60)) {        /* keep the tail visible */
        const char *slash = strrchr(s_current_path, '/');
        if (slash && slash != s_current_path) shown = slash;
    }
    draw_string(4, 3, shown, COLOR_TITLE_FG, COLOR_TITLE_BG, 0);
    char info[20];
    snprintf(info, sizeof(info), "%d", s_file_count);
    draw_string(SCREEN_W - 4 - (int)strlen(info) * 8, 3, info, COLOR_DIR_FG, COLOR_TITLE_BG, 0);
    draw_string(4, 16, "drag=scroll  tap=open  top=up", 0x8410, COLOR_TITLE_BG, 0);

    /* ── File rows ── */
    for (int i = 0; i < visible_rows; i++) {
        int idx = s_scroll_offset + i;
        int y   = TITLE_H + i * ROW_HEIGHT;
        if (idx >= s_file_count) break;
        file_entry_t *fe = &s_files[idx];

        uint16_t bg = (idx == s_selected) ? COLOR_ROW_SEL
                                          : ((i & 1) ? COLOR_ROW_ALT : COLOR_ROW_BG);
        fill_rect(0, y, SCREEN_W - 5, ROW_HEIGHT, bg);

        int ty = y + (ROW_HEIGHT - 16) / 2;   /* vertically center the 16px text */
        if (fe->is_dir) {
            if (strcmp(fe->name, "..") == 0) {
                draw_string(6, ty, "<", COLOR_DIR_FG, bg, 0);
                draw_string(26, ty, "(up one level)", COLOR_DIR_FG, bg, 0);
            } else {
                draw_folder_icon(6, y + (ROW_HEIGHT - 11) / 2, COLOR_DIR_FG, bg);
                draw_string(26, ty, fe->name, COLOR_DIR_FG, bg, 35);
            }
        } else {
            draw_file_icon(6, y + (ROW_HEIGHT - 11) / 2, COLOR_FILE_FG, bg);
            draw_string(26, ty, fe->name, COLOR_FILE_FG, bg, 35);
        }
    }

    /* ── Scrollbar (indicator only; scrolling is by drag) ── */
    fill_rect(SCREEN_W - 5, TITLE_H, 5, list_h, COLOR_SCROLLTRK);
    if (s_file_count > visible_rows) {
        int bar_h = (visible_rows * list_h) / s_file_count;
        if (bar_h < 10) bar_h = 10;
        int span  = list_h - bar_h;
        int bar_y = TITLE_H + (s_scroll_offset * span) / (s_file_count - visible_rows);
        fill_rect(SCREEN_W - 5, bar_y, 5, bar_h, COLOR_SCROLLBAR);
    }

    display_emu_flush();
}

/* Drag-to-scroll state */
static bool s_touching     = false;
static int  s_press_uy     = 0;   /* UI y at touch-down            */
static int  s_press_scroll = 0;   /* scroll offset at touch-down   */
static bool s_dragged      = false;

/* Touch→UI mapping. The 320×240 UI is scaled 3× and rotated 270° by PPA, then
 * drawn at panel offset (0, FB_Y_OFF) on the 720×1280 panel. Touch reports raw
 * panel coords; map them back to UI space. If taps land on the wrong row/axis,
 * tweak here — every tap logs its raw coords for calibration. */
#define FB_Y_OFF  160   /* (1280 - 960) / 2 */
#define FB_SCALE  3
static inline void touch_to_ui(uint16_t px, uint16_t py, int *ux, int *uy)
{
    *ux = ((int)py - FB_Y_OFF) / FB_SCALE;   /* -> 0..320 */
    *uy = (719 - (int)px) / FB_SCALE;        /* -> 0..240 */
}

static void rescan(void)
{
    s_file_count    = scan_directory(s_current_path);
    s_scroll_offset = 0;
    s_selected      = -1;   /* no persistent cursor; tap opens directly */
}

static void go_up(void)
{
    if (strcmp(s_current_path, "/sd") == 0) return;   /* already at root */
    char *slash = strrchr(s_current_path, '/');
    if (slash && slash != s_current_path) {
        *slash = '\0';
        if (strlen(s_current_path) < 3) {
            snprintf(s_current_path, sizeof(s_current_path), "/sd");
        }
        rescan();
    }
}

static void activate_index(int idx)
{
    if (idx < 0 || idx >= s_file_count) return;
    file_entry_t *fe = &s_files[idx];
    if (fe->is_dir) {
        if (strcmp(fe->name, "..") == 0) {
            go_up();
        } else {
            snprintf(s_current_path, sizeof(s_current_path), "%s", fe->full_path);
            rescan();
        }
    } else {
        s_selected = idx;
        s_running  = false;   /* exit run loop → main() launches the ROM */
    }
}

/* Drag to scroll, tap (press+release without much movement) to open/select.
 * Tapping the header strip goes up one directory. */
static void handle_touch(void)
{
    uint16_t px = 0, py = 0;
    bool pressed = bsp_touch_read(&px, &py);

    int visible_rows = (SCREEN_H - TITLE_H) / ROW_HEIGHT;
    int max_off = (s_file_count > visible_rows) ? (s_file_count - visible_rows) : 0;

    if (pressed) {
        int ux, uy;
        touch_to_ui(px, py, &ux, &uy);
        (void)ux;
        if (!s_touching) {                 /* press edge */
            s_touching     = true;
            s_dragged      = false;
            s_press_uy     = uy;
            s_press_scroll = s_scroll_offset;
        } else {                           /* held → drag-scroll */
            int dy = uy - s_press_uy;
            if (dy > 6 || dy < -6) s_dragged = true;
            if (s_dragged) {
                int off = s_press_scroll - dy / ROW_HEIGHT;
                if (off < 0) off = 0;
                if (off > max_off) off = max_off;
                s_scroll_offset = off;
            }
        }
    } else {
        if (s_touching && !s_dragged) {    /* release without drag = tap */
            if (s_press_uy < TITLE_H) {
                go_up();
            } else {
                int idx = s_press_scroll + (s_press_uy - TITLE_H) / ROW_HEIGHT;
                activate_index(idx);
            }
        }
        s_touching = false;
    }
}

/* Native-resolution touch: panel coords map 1:1 to the 720×1280 canvas. */
static void handle_touch_native(void)
{
    uint16_t px = 0, py = 0;
    bool pressed = bsp_touch_read(&px, &py);
    int max_off = (s_file_count > VIS_ROWS) ? (s_file_count - VIS_ROWS) : 0;

    if (pressed) {
        if (!s_touching) {
            s_touching = true; s_dragged = false;
            s_press_uy = py; s_press_scroll = s_scroll_offset;
        } else {
            int dy = (int)py - s_press_uy;
            if (dy > 12 || dy < -12) s_dragged = true;
            if (s_dragged) {
                int off = s_press_scroll - dy / ROWH;
                if (off < 0) off = 0;
                if (off > max_off) off = max_off;
                s_scroll_offset = off;
            }
        }
    } else {
        if (s_touching && !s_dragged) {
            if (s_press_uy < HEADER_H) {
                go_up();
            } else if (s_press_uy < NH - FOOTER_H) {
                int idx = s_press_scroll + (s_press_uy - LIST_Y) / ROWH;
                activate_index(idx);
            }
        }
        s_touching = false;
    }
}

esp_err_t file_browser_init(const char *base_path)
{
    snprintf(s_current_path, sizeof(s_current_path), "%s", base_path);
    s_file_count = scan_directory(s_current_path);
    s_scroll_offset = 0;
    s_selected = -1;
    ESP_LOGI(TAG, "Found %d files in %s", s_file_count, s_current_path);
    return ESP_OK;
}

int file_browser_run(char *selected_path, int max_len)
{
    /* Native 720×1280 portrait canvas, blitted straight to the panel (crisp,
     * no PPA upscale). Allocated once and kept for the session. */
    if (!s_canvas) {
        s_canvas = heap_caps_aligned_calloc(64, 1, (size_t)NW * NH * 2,
                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        if (!s_canvas) { ESP_LOGE(TAG, "canvas alloc failed"); return -1; }
    }

    s_running = true;

    /* Wait for the screen to be released before accepting input. Otherwise the
     * touch still held from the MENU press that exited the previous game is read
     * as an immediate tap and auto-launches whatever row sits under it. */
    s_touching = false;
    s_dragged  = false;
    {
        uint16_t px, py;
        int up = 0;
        while (up < 4) {                 /* ~120ms of continuous no-touch */
            up = bsp_touch_read(&px, &py) ? 0 : (up + 1);
            vTaskDelay(pdMS_TO_TICKS(30));
        }
    }

    while (s_running) {
        draw_ui_native();
        handle_touch_native();
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    if (s_selected >= 0 && s_selected < s_file_count && !s_files[s_selected].is_dir) {
        snprintf(selected_path, max_len, "%s", s_files[s_selected].full_path);
        return 0;
    }
    return -1;
}

void file_browser_deinit(void)
{
    s_running = false;
}

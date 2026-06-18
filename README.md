# tab5-vgbanext — VBA-Next GBA emulator for M5Stack Tab5

A standalone GBA emulator for the M5Stack Tab5 (ESP32-P4), built on the
[libretro/vba-next](https://github.com/libretro/vba-next) core instead of mGBA
(see the sibling `tab5-gba` project for the mGBA + dynarec build).

VBA-Next is the GBA core of choice on low-power handhelds: a compact, plain-C
ARM interpreter with aggressive idle-loop speedhacks. This port reuses the whole
Tab5 harness (display / audio / input / file browser) from `tab5-gba` and adds a
thin libretro frontend.

## Layout

```
vba-next/                 upstream core (git clone, unmodified except a ROM-size patch in src/gba.c)
components/
  vbanext/                ESP-IDF wrapper: compiles vba-next src/*.c + libretro/libretro.c
  vgba_run/               the Tab5 frontend (libretro callbacks -> Tab5 harness)
  bsp audio ppa_engine
  gamepad odroid
  file_browser app_common shared harness, copied from tab5-gba
apps/vgba_tab5/           the application (file picker -> vgba_run)
```

## How the core is driven

VBA-Next is a libretro core. Rather than embed a full libretro frontend, we
compile the core together with its `libretro.c` (which supplies `gba_init`,
`set_memory_maps`, per-game save-type autodetect, and the `system*` hooks) and
implement only the five libretro callbacks in `vgba_run.c`:

| callback            | Tab5 side                                            |
|---------------------|------------------------------------------------------|
| environment         | accept RGB565, decline the rest (defaults are fine)  |
| video refresh       | compact `pix` (stride 256) → 240×160, PPA 3× + DSI   |
| audio sample batch  | accumulate 32 kHz stereo → resample 48 kHz → I2S     |
| input poll / state  | read the on-screen vpad / gamepad → libretro buttons |

Main loop: `poll → retro_run() → blit frame → push audio → check MENU`.

## Build / flash

```powershell
. C:\Espressif\frameworks\esp-idf-v5.5.2\export.ps1
cd apps\vgba_tab5
idf.py set-target esp32p4   # first time only
idf.py -p COM4 -b 460800 build flash
```

ESP-IDF 5.5.2, ESP32-P4, 16 MB flash, 32 MB PSRAM. Put `.gba`/`.zip` ROMs on the
SD card.

## Core build switches (components/vbanext/CMakeLists.txt)

`FRONTEND_SUPPORTS_RGB565=1 LOAD_FROM_MEMORY=1 TILED_RENDERING=0 HAVE_NEON=0`
threaded renderer off. `USE_TWEAKS=0` (the idle-loop speedhacks) — flip to `1`
as a performance lever once the baseline is validated on hardware.

## Notes / known work

- **ROM buffer size.** Upstream allocates a flat 32 MB ROM buffer (open-bus fill
  + a fixed SWI trampoline baked into the HLE BIOS at `rom[0x1fe209c]`). A flat
  32 MB will not fit in 32 MB PSRAM alongside framebuffers — the same overflow
  the mGBA port hit. The fix (relocating the trampoline so the buffer can be
  sized to the actual ROM, like mGBA does) lives in `src/gba.c`; see that file
  and the project memory for status.
- Single-core serial loop for now; the emulate/display split used by `tab5-gba`
  can be layered on later (the core hands back a complete frame).
- No real GBA BIOS needed — VBA-Next uses an HLE BIOS.

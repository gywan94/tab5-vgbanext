# VBA-Next GBA core (libretro/vba-next) wrapped as an ESP-IDF component.
#
# We compile only the emulator core (no libretro frontend); the Tab5 frontend
# lives in the vgba_run component, which provides the system* callbacks the core
# expects (systemDrawScreen / systemOnWriteDataToSoundBuffer / ...).
#
# ROM is handed in from memory (LOAD_FROM_MEMORY), output is RGB565 to match the
# Tab5 PPA/DSI pipeline. NEON and the threaded renderer are off (RISC-V P4).
set(VBA "${CMAKE_CURRENT_LIST_DIR}/../../vba-next")

idf_component_register(
    SRCS
        "${VBA}/src/gba.c"
        "${VBA}/src/memory.c"
        "${VBA}/src/sound.c"
        "${VBA}/src/system.c"
        "${VBA}/src/thread.c"
        "${VBA}/libretro-common/memalign.c"
        "${VBA}/libretro/libretro.c"
    INCLUDE_DIRS
        "${VBA}/src"
        "${VBA}/libretro"
        "${VBA}/libretro-common/include"
    REQUIRES )

# Core build switches (see vba-next Makefile.libretro for the canonical set).
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    INLINE=inline
    FRONTEND_SUPPORTS_RGB565=1
    LOAD_FROM_MEMORY=1
    TILED_RENDERING=0
    USE_TWEAKS=1
    # USE_TWEAK_SPEEDHACK: A/B tested — ZERO gain on koudai (54/17ms) & Yoshi (35/27ms), both
    # identical to without it (USE_TWEAKS already does the idle-loop accel) + it reduces timing
    # accuracy → not worth it. Left off.
    USE_FRAME_SKIP=1
    HAVE_NEON=0
    VGBA_NOSOUND=0          # sound back ON (debugging the JIT soundTicks drift with sound)
    USE_THREADED_RENDERER=1
    THREADED_RENDERER=1                 # the macro the core actually #if's on
    DEFAULT_THREADED_RENDERER_ENABLED=1 # spawn the core1 worker at retro_init
    __LIBRETRO__=1
)

# VBA-Next is third-party, warning-heavy C kept verbatim (except the ROM-size
# patch). -w disables all warnings (so the project-wide -Werror=* can't trip the
# build); -O3 for the ARM interpreter. Mirrors the mGBA component in tab5-gba.
target_compile_options(${COMPONENT_LIB} PRIVATE
    -w
    -Wno-error=all
    -Wno-implicit-function-declaration
    -O3)

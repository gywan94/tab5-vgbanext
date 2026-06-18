idf_component_register(
    SRCS "vgba_jit.c"
    INCLUDE_DIRS "."
    REQUIRES esp_mm hal esp_rom)

# JIT codegen wants speed; warnings off (it does raw RISC-V encoding).
target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -Wno-error=all)

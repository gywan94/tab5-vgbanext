idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES app_common vgba_run vgba_jit bsp gamepad ppa_engine file_browser
)

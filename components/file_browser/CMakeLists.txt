idf_component_register(
    SRCS "file_browser.c" "cjk_font16.c"
    INCLUDE_DIRS "include" "."
    REQUIRES bsp odroid vfs fatfs esp_timer esp_lcd esp_mm
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-format-truncation)

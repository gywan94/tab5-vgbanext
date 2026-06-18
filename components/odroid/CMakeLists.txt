set(ODROID_REQUIRES bsp ppa_engine gamepad audio driver esp_lcd esp_psram fatfs vfs sdmmc nvs_flash esp_partition esp_app_format esp_hw_support esp_mm esp_adc)

idf_component_register(
    SRCS "odroid_display.c" "odroid_input.c" "odroid_settings.c" "odroid_sdcard.c" "odroid_system.c" "odroid_audio.c" "odroid_vpad.c" "vpad_font8.c"
    INCLUDE_DIRS "include"
    REQUIRES ${ODROID_REQUIRES}
    PRIV_REQUIRES esp_timer
)

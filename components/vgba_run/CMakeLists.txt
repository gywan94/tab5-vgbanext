idf_component_register(
    SRCS "vgba_run.c"
    INCLUDE_DIRS "."
    REQUIRES vbanext odroid bsp gamepad ppa_engine audio esp_timer freertos esp_rom esp_partition)

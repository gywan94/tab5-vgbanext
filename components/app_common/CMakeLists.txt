idf_component_register(
    SRCS "app_common.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash app_update esp_partition bsp odroid ppa_engine audio
)

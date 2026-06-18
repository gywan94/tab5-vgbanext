idf_component_register(
    SRCS "audio.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_i2s esp_codec_dev bsp
)

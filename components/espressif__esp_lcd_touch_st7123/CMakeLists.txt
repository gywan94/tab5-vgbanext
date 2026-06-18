idf_component_register(SRCS "esp_lcd_touch_st7123.c" INCLUDE_DIRS "include" REQUIRES "esp_lcd" "esp_lcd_touch")

# These are normally provided by the component manager's cu_pkg_define_version();
# define them here since this is vendored as a local component.
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    ESP_LCD_TOUCH_ST7123_VER_MAJOR=1
    ESP_LCD_TOUCH_ST7123_VER_MINOR=0
    ESP_LCD_TOUCH_ST7123_VER_PATCH=0)

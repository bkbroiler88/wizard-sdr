idf_component_register(
    SRCS "esp_lcd_ili9341.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_lcd
)

# Version macros that the vendored source expects (normally injected by the
# IDF component manager from idf_component.yml). We don't care about the
# number -- just that the symbols exist so ESP_LOGI compiles.
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    ESP_LCD_ILI9341_VER_MAJOR=2
    ESP_LCD_ILI9341_VER_MINOR=0
    ESP_LCD_ILI9341_VER_PATCH=2
)

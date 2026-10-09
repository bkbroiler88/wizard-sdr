idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_timer nvs_flash esp_wifi esp_phy
                  esp_lcd esp_lcd_ili9341
)

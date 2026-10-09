// Signalhound CYD -- ESP-IDF rewrite
// Milestone 2: ILI9341 display using Espressif's esp_lcd_ili9341 driver
// (vendored under components/esp_lcd_ili9341) + the esp_lcd panel API's
// swap_xy / mirror / invert_color operations. No hand-rolled MADCTL.
//
// Pins (CYD 2.8" ESP32-2432S028R):
//   HSPI: MOSI 13, MISO 12, SCLK 14, CS 15, DC 2  (no RST line, so -1)
//   BL (backlight): GPIO 21 (drive high = on)

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ili9341.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "sh";

#define LCD_HOST          SPI2_HOST
#define LCD_SCLK_GPIO     GPIO_NUM_14
#define LCD_MOSI_GPIO     GPIO_NUM_13
#define LCD_MISO_GPIO     GPIO_NUM_12
#define LCD_CS_GPIO       GPIO_NUM_15
#define LCD_DC_GPIO       GPIO_NUM_2
#define LCD_RST_GPIO      (-1)
#define LCD_BL_GPIO       GPIO_NUM_21

// Native portrait, no coord swap -- probe mode. We'll derive the real
// mapping from where the user actually sees the colored corner blocks.
#define LCD_H_RES  240
#define LCD_V_RES  320
#define LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define LCD_CMD_BITS   8
#define LCD_PARAM_BITS 8

#define RGB565(r, g, b)  ((uint16_t)(((r) & 0xF8) << 8 | ((g) & 0xFC) << 3 | ((b) & 0xF8) >> 3))

static esp_lcd_panel_handle_t s_panel = NULL;

static void lcd_init(void)
{
    // Backlight ON
    gpio_config_t bl = {
        .pin_bit_mask = 1ULL << LCD_BL_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&bl);
    gpio_set_level(LCD_BL_GPIO, 1);

    // SPI bus
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_SCLK_GPIO,
        .mosi_io_num = LCD_MOSI_GPIO,
        .miso_io_num = LCD_MISO_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 40 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Panel IO
    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = LCD_DC_GPIO,
        .cs_gpio_num = LCD_CS_GPIO,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &io));

    // Use Espressif's ILI9341 panel driver (vendored under components/).
    // This has the correct init sequence and MADCTL handling for ILI9341.
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_RST_GPIO,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,   // CYD panel is BGR-wired
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io, &panel_cfg, &s_panel));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));

    // No swap_xy -- panel ignores MV on this unit; drawing math goes
    // through the frame as native portrait 240x320.
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, false, false));

    // CYD panel wiring needs color inversion to render true colors.
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, true));

    // Display ON (esp_lcd 5.x API)
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    ESP_LOGI(TAG, "ILI9341 panel init OK (%dx%d landscape)", LCD_H_RES, LCD_V_RES);
}

// Byte-swapped RGB565 blit at raw native (CASET, RASET) coords.
static void fill_rect(int x, int y, int w, int h, uint16_t color)
{
    static uint16_t row[LCD_H_RES];
    uint16_t swapped = (uint16_t)((color >> 8) | (color << 8));
    for (int i = 0; i < w && i < LCD_H_RES; i++) row[i] = swapped;
    for (int yy = y; yy < y + h; yy++) {
        esp_lcd_panel_draw_bitmap(s_panel, x, yy, x + w, yy + 1, row);
    }
}

static void draw_test_pattern(void)
{
    // ORIENTATION PROBE. Fill everything black, then paint 4 BIG colored
    // blocks (60x60) at the four corners of our code's coord space:
    //   (x=0,    y=0)    -> RED
    //   (x=180,  y=0)    -> GREEN
    //   (x=0,    y=260)  -> BLUE
    //   (x=180,  y=260)  -> WHITE
    // Code's coords are (x in 0..LCD_H_RES-1=239, y in 0..LCD_V_RES-1=319).
    // Then tell me WHERE on the physical panel each color actually shows up.
    fill_rect(0, 0, LCD_H_RES, LCD_V_RES, RGB565(0, 0, 0));

    const int sz = 60;
    fill_rect(0,              0,              sz, sz, RGB565(255,   0,   0));  // code TL -> RED
    fill_rect(LCD_H_RES - sz, 0,              sz, sz, RGB565(  0, 255,   0));  // code TR -> GREEN
    fill_rect(0,              LCD_V_RES - sz, sz, sz, RGB565(  0,   0, 255));  // code BL -> BLUE
    fill_rect(LCD_H_RES - sz, LCD_V_RES - sz, sz, sz, RGB565(255, 255, 255));  // code BR -> WHITE
}

void app_main(void)
{
    ESP_LOGI(TAG, "Signalhound IDF rewrite -- M2 boot (ILI9341 via esp_lcd component)");
    lcd_init();
    draw_test_pattern();
    ESP_LOGI(TAG, "Test pattern drawn");

    int heartbeat = 0;
    for (;;) {
        ESP_LOGI(TAG, "alive heartbeat=%d free=%u",
                 heartbeat++, (unsigned)esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

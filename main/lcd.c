#include "lcd.h"

volatile bool lcd_busy = false;

bool finish_flush(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    lcd_busy = false;
    return true;
}


esp_lcd_panel_handle_t lcd_panel_handle = NULL;


esp_err_t init_lcd() {
    ESP_LOGI(LCD_LOG_TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << ST7789_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_LOGI(LCD_LOG_TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
            .sclk_io_num = ST7789_PIN_NUM_SCLK,
            .mosi_io_num = ST7789_PIN_NUM_MOSI,
            .miso_io_num = ST7789_PIN_NUM_MISO,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 240 * 240 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(LCD_LOG_TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
            .dc_gpio_num = ST7789_PIN_NUM_LCD_DC,
            .cs_gpio_num = ST7789_PIN_NUM_LCD_CS,
            .pclk_hz = SPI_MASTER_FREQ_40M,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .spi_mode = 2,
            .trans_queue_depth = 40,
            .on_color_trans_done = finish_flush,
//            .user_ctx = disp_drv,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) LCD_HOST, &io_config, &io_handle));

    ESP_LOGI(LCD_LOG_TAG, "Install ST7789 panel driver");
    esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = ST7789_PIN_NUM_LCD_RST,
            .rgb_endian = LCD_RGB_ENDIAN_RGB,
            .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &lcd_panel_handle));

    //ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle)); //disabled reset here, delay not enough, need fixup.
    gpio_set_level(ST7789_PIN_NUM_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(ST7789_PIN_NUM_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(ST7789_PIN_NUM_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel_handle));

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel_handle, true));

    ESP_LOGI(LCD_LOG_TAG, "Turn on LCD backlight");
    gpio_set_level(ST7789_PIN_NUM_BK_LIGHT, ST7789_LCD_BK_LIGHT_ON_LEVEL);
    esp_lcd_panel_invert_color(lcd_panel_handle, true);
    return ESP_OK;
}
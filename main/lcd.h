#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#define LCD_LOG_TAG "${PROJECT_NAME}:lcd"

#define LCD_HOST  SPI2_HOST

#define ST7789_LCD_BK_LIGHT_ON_LEVEL  1
#define ST7789_PIN_NUM_SCLK           18
#define ST7789_PIN_NUM_MOSI           17
#define ST7789_PIN_NUM_MISO           (-1)
#define ST7789_PIN_NUM_LCD_DC         15
#define ST7789_PIN_NUM_LCD_RST        16
#define ST7789_PIN_NUM_LCD_CS         (-1)
#define ST7789_PIN_NUM_BK_LIGHT       14

#define LCD_WIDTH       240
#define LCD_HEIGHT      240


extern volatile bool lcd_busy;
extern esp_lcd_panel_handle_t lcd_panel_handle;

esp_err_t init_lcd();

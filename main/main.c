#include <sys/cdefs.h>
#include <esp_log.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "apriltag.h"
#include "tag16h5.h"
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "timeProbe.h"
#include "lcd.h"
#include "camera.h"
//#include "utils.h"
#include "taskMonitor.h"
#include "apriltag_detection.h"


#define MAIN_LOG_TAG "cube01:main"


EXT_RAM_BSS_ATTR uint16_t frameBuffer[240 * 240];


_Noreturn void app_main() {
    startTaskMonitor(10000);
    //初始化st7789屏幕
    if (ESP_OK != init_lcd()) {
        ESP_LOGE(MAIN_LOG_TAG, "LCD init fail");
        while (1);
    }
    //初始化摄像头
    if (ESP_OK != init_camera()) {
        ESP_LOGE(MAIN_LOG_TAG, "LCD init fail");
        while (1);
    }
    //初始化
    apriltag_init();


    timeProbe_t camera;
    timeProbe_t screen;
    timeProbe_t fps;

    while (1) {
        timeProbe_start(&camera);
        timeProbe_start(&fps);
        camera_fb_t *pic = esp_camera_fb_get();
        // use pic->buf to access the image
        ESP_LOGI(MAIN_LOG_TAG, "Picture taken! Its size was: %zu bytes", pic->len);

//摄像头数据复制到显存，同时旋转屏幕
        {
//        memcpy(frameBuffer, pic->buf, 240 * 240 * 2);
            for (int y = 0; y < LCD_HEIGHT; y++) {
                for (int x = 0; x < LCD_WIDTH; x++) {
                    // 计算旋转后的位置
                    int new_x = (LCD_WIDTH - 1) - y;
                    int new_y = x;
                    // 复制像素到旋转后的位置
                    ((uint16_t *) frameBuffer)[new_y * LCD_WIDTH + new_x] = ((uint16_t *) pic->buf)[y * LCD_WIDTH + x];
                }
            }
        }



//        for (int i = 0; i < 96; ++i) {
//            memcpy(&WorkFrame[CONFIG_WIDTH*i],&((pic->buf)[i*96*2]),96*2);
//        }
        esp_camera_fb_return(pic);
        ESP_LOGI(MAIN_LOG_TAG, "Picture taken! cost time: %lld us", timeProbe_stop(&camera));


        apriltag_find(frameBuffer);


        timeProbe_start(&screen);
        while (lcd_busy) {
            vTaskDelay(pdMS_TO_TICKS(2));
            ESP_LOGE(MAIN_LOG_TAG, "wait");

        }
        lcd_busy = true;
        esp_lcd_panel_draw_bitmap(lcd_panel_handle, 0, 0, LCD_WIDTH, LCD_HEIGHT, frameBuffer);

        ESP_LOGI(MAIN_LOG_TAG, "full screen refresh : %lld us", timeProbe_stop(&screen));
        ESP_LOGI(MAIN_LOG_TAG, "fps: %f", 1000 / (timeProbe_stop(&fps) / 1000.0));
        ESP_LOGI(MAIN_LOG_TAG, "esp_get_free_heap_size: %lu", esp_get_free_heap_size());
        ESP_LOGI(MAIN_LOG_TAG, "esp_get_free_internal_heap_size: %lu", esp_get_free_internal_heap_size());
    }
    apriltag_destroy();
}
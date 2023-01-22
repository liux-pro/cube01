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
#include "utils.h"
#include "taskMonitor.h"


#define MAIN_LOG_TAG "${PROJECT_NAME}:main"


uint16_t frameBuffer[240 * 240];
EXT_RAM_BSS_ATTR uint8_t gray[240 * 240];


_Noreturn void app_main() {
    startTaskMonitor(10000);
    //初始化st7789屏幕
    if (ESP_OK != init_lcd()) {
        return;
    }
    //初始化摄像头
    if (ESP_OK != init_camera()) {
        return;
    }


    timeProbe_t camera;
    timeProbe_t screen;
    timeProbe_t fps;
    timeProbe_t gray_convert;
    apriltag_family_t *tf = NULL;
    tf = tag16h5_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family_bits(td, tf, 1);
    // 降低分辨率，会提高速度，降低精度 eg. 2 减小两倍
    td->quad_decimate = 2;
    //高斯模糊 eg 0.8
    td->quad_sigma = 0.8f;
    // 线程数
    td->nthreads = 1;
    //
    td->debug = false;
    td->refine_edges = true;


    image_u8_t *im = NULL;

    static image_u8_t tmp = {
            .width=240,
            .height=240,
            .stride=240,
    };
    tmp.buf = gray;
    im = &tmp;


    while (1) {
        timeProbe_start(&camera);
        timeProbe_start(&fps);
        camera_fb_t *pic = esp_camera_fb_get();
        // use pic->buf to access the image
        ESP_LOGI(MAIN_LOG_TAG, "Picture taken! Its size was: %zu bytes", pic->len);

        memcpy(frameBuffer, pic->buf, 240 * 240 * 2);

//        for (int i = 0; i < 96; ++i) {
//            memcpy(&WorkFrame[CONFIG_WIDTH*i],&((pic->buf)[i*96*2]),96*2);
//        }
        esp_camera_fb_return(pic);
        ESP_LOGI(MAIN_LOG_TAG, "Picture taken! cost time: %lld us", timeProbe_stop(&camera));

        //////////////



        timeProbe_start(&gray_convert);
        convert_rgb565_to_grayscale(frameBuffer, gray, 240, 240);
        ESP_LOGI(MAIN_LOG_TAG, "gray: %lld", timeProbe_stop(&gray_convert));

        zarray_t *detections = apriltag_detector_detect(td, im);
        for (int i = 0; i < zarray_size(detections); i++) {
            int x, y;
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            ESP_LOGE("apriltag", "detection %3d: id (%2lux%2lu)-%-4d, hamming %d, margin %8.3f\n",
                     i, det->family->nbits, det->family->h, det->id, det->hamming, det->decision_margin);
            ESP_LOGE("apriltag", "x %8.3lf   y %8.3lf\n",
                     det->c[0], det->c[1]);
            x = (int)det->c[0];
            y = (int)det->c[1];
            if (x + y != 0) {
                draw_rect_rgb565(frameBuffer, 240, 240, x, y, x + 10, y + 10, swap_uint16(0b1111100000000000));
            }
        }
        apriltag_detections_destroy(detections);


        timeProbe_start(&screen);

        lcd_busy = true;
        esp_lcd_panel_draw_bitmap(lcd_panel_handle, 0, 0, 240, 240, frameBuffer);
        while (lcd_busy) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        ESP_LOGI(MAIN_LOG_TAG, "full screen refresh : %lld us", timeProbe_stop(&screen));
        ESP_LOGI(MAIN_LOG_TAG, "fps: %f", 1000 / (timeProbe_stop(&fps) / 1000.0));
    }
    apriltag_detector_destroy(td);
    tag16h5_destroy(tf);
}
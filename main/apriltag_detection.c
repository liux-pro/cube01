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

#include "timeProbe.h"
#include "apriltag_detection.h"
#include "utils.h"

#define CALIBRATION_LOG_TAG "cube01:apriltag"


static timeProbe_t gray_convert;
static apriltag_family_t *tf = NULL;
static apriltag_detector_t *apriltag_detector = NULL;
static image_u8_t *p_img = NULL;
static EXT_RAM_BSS_ATTR uint8_t gray[APRILTAG_HEIGHT * APRILTAG_WIDTH];

void apriltag_init() {
    tf = tag16h5_create();
    apriltag_detector = apriltag_detector_create();
    apriltag_detector_add_family_bits(apriltag_detector, tf, 1);
    // 降低分辨率，会提高速度，降低精度 eg. 2 减小两倍
    apriltag_detector->quad_decimate = 2;
    //高斯模糊 eg 0.8
    apriltag_detector->quad_sigma = 0.8f;
    // 线程数
    apriltag_detector->nthreads = 2;
    //
    apriltag_detector->debug = false;
    apriltag_detector->refine_edges = true;


    static image_u8_t img = {
            .width=APRILTAG_WIDTH,  //图片宽度
            .height=APRILTAG_HEIGHT,  //图片高度
            .stride=APRILTAG_WIDTH,   //每行的实际占的字节数。因为有些系统会有padding，例如图像宽度是7，但是一行实际占用8个字节，这里就填8
    };
    img.buf = gray;
    p_img = &img;
}

void apriltag_find(uint16_t *frameBuffer) {

    timeProbe_start(&gray_convert);
    convert_rgb565_to_grayscale(frameBuffer, gray, 240, 240);
    ESP_LOGI(CALIBRATION_LOG_TAG, "gray: %lld", timeProbe_stop(&gray_convert));

    zarray_t *detections = apriltag_detector_detect(apriltag_detector, p_img);
    for (int i = 0; i < zarray_size(detections); i++) {
        int x, y;
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        ESP_LOGE("apriltag", "detection %3d: id (%2lux%2lu)-%-4d, hamming %d, margin %8.3f\n",
                 i, det->family->nbits, det->family->h, det->id, det->hamming, det->decision_margin);
        ESP_LOGE("apriltag", "x %8.3lf   y %8.3lf\n",
                 det->c[0], det->c[1]);
        x = (int) det->c[0];
        y = (int) det->c[1];
        if (x + y != 0) {
            draw_rect_rgb565(frameBuffer, 240, 240, x, y, x + 10, y + 10, swap_uint16(0b1111100000000000));
        }
    }
    apriltag_detections_destroy(detections);
}
void apriltag_destroy() {
    apriltag_detector_destroy(apriltag_detector);
    tag16h5_destroy(tf);
}

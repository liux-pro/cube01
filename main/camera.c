#include "camera.h"


static camera_config_t camera_config = {
        .pin_pwdn = -1,
        .pin_reset = 9,
        .pin_xclk = 12,
        .pin_sccb_sda = 3,
        .pin_sccb_scl = 10,

        .pin_d7 = 42,
        .pin_d6 = 41,
        .pin_d5 = 40,
        .pin_d4 = 39,
        .pin_d3 = 47,
        .pin_d2 = 1,
        .pin_d1 = 21,
        .pin_d0 = 38,
        .pin_vsync = 17,
        .pin_href = 16,
        .pin_pclk = 11,

        //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
        .xclk_freq_hz = 25000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_RGB565, //YUV422,GRAYSCALE,RGB565,JPEG
        .frame_size = FRAMESIZE_240X240,    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

        .jpeg_quality = 12, //0-63, for OV series camera sensors, lower number means higher quality
        .fb_count = 2,       //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

esp_err_t init_camera() {
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(CAMERA_LOG_TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

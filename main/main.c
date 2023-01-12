#include <esp_log.h>
#include <string.h>
#include "st7789_faster.h"
#include "timeProbe.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include "esp_camera.h"
#include "esp_heap_caps.h"


static const char *TAG = "example:take_picture";

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
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_RGB565, //YUV422,GRAYSCALE,RGB565,JPEG
        .frame_size = FRAMESIZE_240X240,    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

        .jpeg_quality = 12, //0-63, for OV series camera sensors, lower number means higher quality
        .fb_count = 2,       //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};


static esp_err_t init_camera() {
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}



TFT_t dev;

uint16_t WorkFrame[240 * 240];




spi_transaction_t p_spi_transaction_pool[30];

void initSpiTransactionPool() {
    for (int i = 0; i < 30; i += 1) {
        p_spi_transaction_pool[i].length = CONFIG_WIDTH * 2 * 8*8;
        p_spi_transaction_pool[i].tx_buffer = &WorkFrame[i * (CONFIG_WIDTH)*8];
    }
}


void app_main() {
    gpio_reset_pin(13);
    gpio_set_direction(13,GPIO_MODE_OUTPUT);
    gpio_set_level(13,0);
    gpio_set_level(13,1);
//    vTaskDelay(3000 / portTICK_RATE_MS);

    spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO,
                    CONFIG_BL_GPIO);

    lcdInit(&dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);
//    vTaskDelay(1000 / portTICK_RATE_MS);

    initSpiTransactionPool();
    if (ESP_OK != init_camera()) {
        return;
    }

//    sensor_t *s = esp_camera_sensor_get();
//    s->set_lenc(s,0);
//    s->set_raw_gma(s,0);
//    s->set_wpc(s,0);
//    s->set_gain_ctrl(s,0);
//    s->set_exposure_ctrl(s,0);
//    s->set_awb_gain(s,0);
//    s->set_whitebal(s,0);
//    s->set_brightness(s,0);
//    s->set_contrast(s,0);
//    s->set_saturation(s,0);
//    s->set_ae_level(s,0);
//    s->set_aec2(s,0);
////    曝光度（0-1200）
//    s->set_aec_value(s,500);

    timeProbe_t camera;
    timeProbe_t screen;
    timeProbe_t screen_prepare;
    while (1) {
        timeProbe_start(&camera);
        camera_fb_t *pic = esp_camera_fb_get();
        // use pic->buf to access the image
        ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes", pic->len);
        memcpy(WorkFrame,pic->buf,240*240*2);

//        for (int i = 0; i < 96; ++i) {
//            memcpy(&WorkFrame[CONFIG_WIDTH*i],&((pic->buf)[i*96*2]),96*2);
//        }
        esp_camera_fb_return(pic);
        ESP_LOGI(TAG, "Picture taken! cost time: %lld", timeProbe_stop(&camera));

        //////////////


        gpio_set_level(13,0);

        timeProbe_start(&screen_prepare);
        lcdPrepareMultiPixels(&dev);
        ESP_LOGI(TAG, "screen_prepare: %lld", timeProbe_stop(&screen_prepare));
        timeProbe_start(&screen);

        for (int i = 0; i < 30; i += 1) {
            spi_device_queue_trans(dev._SPIHandle, &p_spi_transaction_pool[i], portMAX_DELAY);
        }

        spi_transaction_t *r_trans;
        for (int i = 0; i < 30; i += 1) {
            spi_device_get_trans_result(dev._SPIHandle, &r_trans, portMAX_DELAY);
        }

        ESP_LOGI(TAG, "screen: %lld", timeProbe_stop(&screen_prepare));
    vTaskDelay(100 / portTICK_PERIOD_MS);

        //////////////////
        gpio_set_level(13,1);
    }
}

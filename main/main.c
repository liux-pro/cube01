#include <esp_log.h>
#include <string.h>
#include "timeProbe.h"
#include "delay.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "apriltag.h"
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_heap_caps.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

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
        .xclk_freq_hz = 30000000,
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



uint16_t frameBuffer[240 * 240];
EXT_RAM_BSS_ATTR uint8_t gray[240 * 240];

uint16_t swap_uint16(uint16_t val) {
    return (val << 8) | (val >> 8);
}

void convert_rgb565_to_grayscale(const unsigned short *rgb, unsigned char *gray, int width, int height) {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            unsigned short pixel = swap_uint16(rgb[y * width + x]);
            unsigned char red = (pixel >> 11) & 0x1F;
            unsigned char green = (pixel >> 5) & 0x3F;
            unsigned char blue = pixel & 0x1F;
//            gray[y * width + x] = (unsigned char)(0.2989 * red + 0.5870 * green + 0.1140 * blue);
//           convert to fixed point
            gray[y * width + x] = (red * 19595 + green * 38469 + blue * 7472) >> 16;
        }
    }
}

/**
 *
 * @param screen
 * @param width
 * @param height
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @param color
 */
void
draw_rect_rgb565(unsigned short *screen, int width, int height, int x1, int y1, int x2, int y2, unsigned short color) {
    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            if (x >= 0 && x < width && y >= 0 && y < height) {
                screen[y * width + x] = color;
            }
        }
    }
}


spi_transaction_t p_spi_transaction_pool[30];




#define LCD_HOST  SPI2_HOST

#define ST7789_LCD_BK_LIGHT_ON_LEVEL  1
#define ST7789_PIN_NUM_SCLK           4
#define ST7789_PIN_NUM_MOSI           5
#define ST7789_PIN_NUM_MISO           (-1)
#define ST7789_PIN_NUM_LCD_DC         7
#define ST7789_PIN_NUM_LCD_RST        6
#define ST7789_PIN_NUM_LCD_CS         (-1)
#define ST7789_PIN_NUM_BK_LIGHT       15


volatile bool fuck = false;

void finish_flush() {
    fuck = true;
}

static esp_lcd_panel_handle_t panel_handle = NULL;

void app_main() {
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << ST7789_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
            .sclk_io_num = ST7789_PIN_NUM_SCLK,
            .mosi_io_num = ST7789_PIN_NUM_MOSI,
            .miso_io_num = ST7789_PIN_NUM_MISO,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 240 * 240 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
            .dc_gpio_num = ST7789_PIN_NUM_LCD_DC,
            .cs_gpio_num = ST7789_PIN_NUM_LCD_CS,
            .pclk_hz = SPI_MASTER_FREQ_80M,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .spi_mode = 2,
            .trans_queue_depth = 40,
            .on_color_trans_done = finish_flush,
//            .user_ctx = disp_drv,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) LCD_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install ST7789 panel driver");
    esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = ST7789_PIN_NUM_LCD_RST,
            .rgb_endian = LCD_RGB_ENDIAN_RGB,
            .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    //ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle)); //disabled reset here, delay not enough, need fixup.
    gpio_set_level(ST7789_PIN_NUM_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(ST7789_PIN_NUM_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(ST7789_PIN_NUM_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(ST7789_PIN_NUM_BK_LIGHT, ST7789_LCD_BK_LIGHT_ON_LEVEL);
    uint8_t aa[10 * 10 * 2] = {};
    esp_lcd_panel_invert_color(panel_handle, true);
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 10, 10, aa);
//!!!!!!!!!!!!!!!



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
    td->quad_sigma = 0.8;
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
        ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes", pic->len);

        memcpy(frameBuffer, pic->buf, 240 * 240 * 2);

//        for (int i = 0; i < 96; ++i) {
//            memcpy(&WorkFrame[CONFIG_WIDTH*i],&((pic->buf)[i*96*2]),96*2);
//        }
        esp_camera_fb_return(pic);
        ESP_LOGI(TAG, "Picture taken! cost time: %lld us", timeProbe_stop(&camera));

        //////////////



        timeProbe_start(&gray_convert);
        convert_rgb565_to_grayscale(frameBuffer, gray, 240, 240);
        ESP_LOGI(TAG, "gray: %lld", timeProbe_stop(&gray_convert));

        zarray_t *detections = apriltag_detector_detect(td, im);
        int x=0,y=0;
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            ESP_LOGE("apriltag","detection %3d: id (%2lux%2lu)-%-4d, hamming %d, margin %8.3f\n",
                     i, det->family->nbits, det->family->h, det->id, det->hamming, det->decision_margin);
            ESP_LOGE("apriltag","x %8.3lf   y %8.3lf\n",
                     det->c[0], det->c[1]);
            x=det->c[0];
            y=det->c[1];
        }
        apriltag_detections_destroy(detections);
        if (x+y != 0){
            draw_rect_rgb565(frameBuffer, 240, 240, x, y, x + 10, y + 10, 0xffff);
        }

        timeProbe_start(&screen);

        fuck = false;
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 240, 240, frameBuffer);
        while (fuck == false) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        ESP_LOGI(TAG, "full screen refresh : %lld us", timeProbe_stop(&screen));
        ESP_LOGI(TAG, "fps: %f", 1000 / (timeProbe_stop(&fps) / 1000.0));
    }
    apriltag_detector_destroy(td);
    tag16h5_destroy(tf);
}
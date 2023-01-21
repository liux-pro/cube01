#include <sys/cdefs.h>
#include <esp_log.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "apriltag.h"
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
#include "esp_lvgl_port.h"
#include <demos/widgets/lv_demo_widgets.h>

#define MAIN_LOG_TAG "${PROJECT_NAME}:main"


//uint16_t frameBuffer[240 * 240];
//EXT_RAM_BSS_ATTR uint8_t gray[240 * 240];


static lv_disp_t * disp;
/* The component calls esp_lcd_panel_draw_bitmap API for send data to the screen. There must be called
lvgl_port_flush_ready(disp) after each transaction to display. The best way is to use on_color_trans_done
callback from esp_lcd IO config structure. */
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_t ** disp = (lv_disp_t **)user_ctx;
    lvgl_port_flush_ready(*disp);
    return false;
}


void LVGL_CentralButton(void) {
    lv_obj_t *btn = lv_btn_create(lv_scr_act());
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_height(btn, 30);

    lv_obj_t *label;
    label = lv_label_create(btn);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(label, "aoifhiaifhaioefieaio");

    static lv_style_t style_btn;
    lv_style_init(&style_btn);
    lv_style_set_radius(&style_btn, 10);
    lv_style_set_border_color(&style_btn, lv_color_white());
    lv_style_set_border_opa(&style_btn, LV_OPA_30);
    lv_obj_add_style(btn, &style_btn, LV_STATE_DEFAULT);
}

void app_main(){
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    esp_err_t err = lvgl_port_init(&lvgl_cfg);
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
            .pclk_hz = SPI_MASTER_FREQ_80M,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .spi_mode = 2,
            .trans_queue_depth = 40,
            .on_color_trans_done = notify_lvgl_flush_ready,
            .user_ctx = &disp
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
    /////////////////////
    /* Add LCD screen */
    const lvgl_port_display_cfg_t disp_cfg = {
            .io_handle = io_handle,
            .panel_handle = lcd_panel_handle,
            .buffer_size = 240*240,
            .double_buffer = true,
            .hres = 240,
            .vres = 240,
            .monochrome = false,
            /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
            .rotation = {
                    .swap_xy = false,
                    .mirror_x = false,
                    .mirror_y = false,
            },
            .flags = {
                    .buff_dma = true,
            }
    };
    disp = lvgl_port_add_disp(&disp_cfg);

    lvgl_port_lock(0);

    LVGL_CentralButton();


    /* Screen operation done -> release for the other task */
    lvgl_port_unlock();

}
//_Noreturn void app_main() {
//    //初始化st7789屏幕
//    if (ESP_OK != init_lcd()) {
//        return;
//    }
//    //初始化摄像头
//    if (ESP_OK != init_camera()) {
//        return;
//    }
//
//
//    timeProbe_t camera;
//    timeProbe_t screen;
//    timeProbe_t fps;
//    timeProbe_t gray_convert;
//    apriltag_family_t *tf = NULL;
//    tf = tag16h5_create();
//    apriltag_detector_t *td = apriltag_detector_create();
//    apriltag_detector_add_family_bits(td, tf, 1);
//    // 降低分辨率，会提高速度，降低精度 eg. 2 减小两倍
//    td->quad_decimate = 2;
//    //高斯模糊 eg 0.8
//    td->quad_sigma = 0.8f;
//    // 线程数
//    td->nthreads = 1;
//    //
//    td->debug = false;
//    td->refine_edges = true;
//
//
//    image_u8_t *im = NULL;
//
//    static image_u8_t tmp = {
//            .width=240,
//            .height=240,
//            .stride=240,
//    };
//    tmp.buf = gray;
//    im = &tmp;
//
//
//    while (1) {
//        timeProbe_start(&camera);
//        timeProbe_start(&fps);
//        camera_fb_t *pic = esp_camera_fb_get();
//        // use pic->buf to access the image
//        ESP_LOGI(MAIN_LOG_TAG, "Picture taken! Its size was: %zu bytes", pic->len);
//
//        memcpy(frameBuffer, pic->buf, 240 * 240 * 2);
//
////        for (int i = 0; i < 96; ++i) {
////            memcpy(&WorkFrame[CONFIG_WIDTH*i],&((pic->buf)[i*96*2]),96*2);
////        }
//        esp_camera_fb_return(pic);
//        ESP_LOGI(MAIN_LOG_TAG, "Picture taken! cost time: %lld us", timeProbe_stop(&camera));
//
//        //////////////
//
//
//
//        timeProbe_start(&gray_convert);
//        convert_rgb565_to_grayscale(frameBuffer, gray, 240, 240);
//        ESP_LOGI(MAIN_LOG_TAG, "gray: %lld", timeProbe_stop(&gray_convert));
//
//        zarray_t *detections = apriltag_detector_detect(td, im);
//        for (int i = 0; i < zarray_size(detections); i++) {
//            int x, y;
//            apriltag_detection_t *det;
//            zarray_get(detections, i, &det);
//            ESP_LOGE("apriltag", "detection %3d: id (%2lux%2lu)-%-4d, hamming %d, margin %8.3f\n",
//                     i, det->family->nbits, det->family->h, det->id, det->hamming, det->decision_margin);
//            ESP_LOGE("apriltag", "x %8.3lf   y %8.3lf\n",
//                     det->c[0], det->c[1]);
//            x = (int)det->c[0];
//            y = (int)det->c[1];
//            if (x + y != 0) {
//                draw_rect_rgb565(frameBuffer, 240, 240, x, y, x + 10, y + 10, swap_uint16(0b1111100000000000));
//            }
//        }
//        apriltag_detections_destroy(detections);
//
//
//        timeProbe_start(&screen);
//
//        lcd_busy = true;
//        esp_lcd_panel_draw_bitmap(lcd_panel_handle, 0, 0, 240, 240, frameBuffer);
//        while (lcd_busy) {
//            vTaskDelay(pdMS_TO_TICKS(2));
//        }
//        ESP_LOGI(MAIN_LOG_TAG, "full screen refresh : %lld us", timeProbe_stop(&screen));
//        ESP_LOGI(MAIN_LOG_TAG, "fps: %f", 1000 / (timeProbe_stop(&fps) / 1000.0));
//    }
//    apriltag_detector_destroy(td);
//    tag16h5_destroy(tf);
//}
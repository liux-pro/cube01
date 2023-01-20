#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_camera.h"

#define CAMERA_LOG_TAG "${PROJECT_NAME}:camera"

esp_err_t init_camera();
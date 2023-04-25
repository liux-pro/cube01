#pragma once

#include "stdint.h"

uint16_t swap_uint16(uint16_t val);

void draw_rect_rgb565(uint16_t *screen, int width, int height, int x1, int y1, int x2, int y2, uint16_t color);

void convert_rgb565_to_grayscale(const uint16_t *rgb, unsigned char *gray, int width, int height);
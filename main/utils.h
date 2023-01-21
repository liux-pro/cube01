#pragma once

#include "stdint.h"

uint16_t swap_uint16(uint16_t val) {
    return (val << 8) | (val >> 8);
}

void draw_rect_rgb565(uint16_t *screen, int width, int height, int x1, int y1, int x2, int y2, uint16_t color) {
    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            if (x >= 0 && x < width && y >= 0 && y < height) {
                screen[y * width + x] = color;
            }
        }
    }
}

uint16_t d_r = (uint16_t) (0.2989 * (1 << 16));
uint16_t d_g = (uint16_t) (0.5870 * (1 << 16));
uint16_t d_b = (uint16_t) (0.1140 * (1 << 16));

void convert_rgb565_to_grayscale(const uint16_t *rgb, unsigned char *gray, int width, int height) {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            uint16_t pixel = swap_uint16(rgb[y * width + x]);
            uint8_t red = (pixel >> 11) & 0x1F;
            uint8_t green = (pixel >> 5) & 0x3F;
            uint8_t blue = pixel & 0x1F;
//            gray[y * width + x] = (unsigned char)(0.2989 * red + 0.5870 * green + 0.1140 * blue);
//          Optimization into fixed point calculation
            gray[y * width + x] = (uint8_t) ((d_r * red + d_g * green + d_b * blue) >> 16);
        }
    }
}

#pragma once

#include "stdint.h"

uint16_t swap_uint16(uint16_t val) {
    return (val << 8) | (val >> 8);
}

void draw_rect_rgb565(unsigned short *screen, int width, int height, int x1, int y1, int x2, int y2, unsigned short color) {
    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            if (x >= 0 && x < width && y >= 0 && y < height) {
                screen[y * width + x] = color;
            }
        }
    }
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

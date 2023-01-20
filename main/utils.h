#pragma once

#include "stdint.h"

uint16_t swap_uint16(uint16_t val) {
    return (val << 8) | (val >> 8);
}
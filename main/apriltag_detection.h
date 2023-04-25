#include "stdint.h"
#define APRILTAG_HEIGHT 240
#define APRILTAG_WIDTH 240

void apriltag_init();
void apriltag_find(uint16_t *frameBuffer);
void apriltag_destroy();
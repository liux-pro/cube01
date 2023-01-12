#include <string.h>
#include <driver/gpio.h>
#include "st7789_faster.h"

/*
 * 设置好lcd内存写入指针，准备快速写入数据
 */
void lcdPrepareMultiPixels(TFT_t *dev) {
    uint16_t _x1 = 0 + dev->_offsetx;
    uint16_t _x2 = dev->_width-1 + dev->_offsetx;
    uint16_t _y1 = 0 + dev->_offsety;
    uint16_t _y2 = dev->_height-1 + dev->_offsety;

    spi_master_write_command(dev, 0x2A);    // set column(x) address
    spi_master_write_addr(dev, _x1, _x2);
    spi_master_write_command(dev, 0x2B);    // set Page(y) address
    spi_master_write_addr(dev, _y1, _y2);
    spi_master_write_command(dev, 0x2C);    //	Memory Write
    gpio_set_level(dev->_dc, 1);
}
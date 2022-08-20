#pragma once

#include <stdlib.h>
#include <memory>
#include <string>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "ScrollBuffer.hpp"

#define LCD_LONG_WIDTH 240
#define LCD_SHORT_WIDTH 135

#define rgb16(r, g, b) ((uint16_t)((0xf800 & ((r) << 8)) | \
                                   (0x07e0 & ((g) << 3)) | \
                                   (0x001f & ((b) >> 3))))

#define color(hex) ((uint16_t)((((hex) >> 8) & 0xf800) | \
                               (((hex) >> 5) & 0x07e0) | \
                               (((hex) >> 3)& 0x001f)))

#define RED color(0xff0000)
#define GREEN color(0x00ff00)
#define BLUE color(0x0000ff)
#define YELLOW color(0xffff00)
#define CYAN color(0x00ffff)
#define MAGENTA color(0xff00ff)
#define WHITE 0xffff
#define BLACK 0
#define GRAY color(0x808080)
#define DARK_GRAY color(0x404040)

typedef enum
{
    PORTRAIT,
    ROTATED_PORTRAIT,
    LANDSCAPE,
    ROTATED_LANDSCAPE
} lcd_orientation_t;

typedef struct {
    spi_host_device_t spi_host;
    gpio_num_t pin_spi_mosi;
    gpio_num_t pin_spi_clock;
    gpio_num_t pin_spi_cs;
    gpio_num_t pin_dc;
    gpio_num_t pin_reset;
    gpio_num_t pin_backlight;
} lcd_init_config_t;

typedef struct {
    int x, y;
} point_t;

const point_t ZERO_XY = { 0, 0 };

typedef struct {
    unsigned int width, height;
} box_size_t;

class Lcd
{
    struct PrintControl;
    lcd_orientation_t orientation;
    box_size_t size;
    int xOffset;
    int yOffset;
    int bufferIndex;
    spi_device_handle_t device;
    gpio_num_t dcPin;
    std::unique_ptr<PrintControl> printControl;
    void send(const void *, size_t, bool);
    void send(uint16_t);
    void flush();
    void addressSetX(int, int);
    void addressSetY(int, int);
    void setWriteFrame(point_t, box_size_t);
public:
    Lcd(lcd_init_config_t config, lcd_orientation_t orientation = LANDSCAPE);
    ~Lcd();
    void displayOn();
    void brightness(int);
    box_size_t getSize();
    void setOrientation(lcd_orientation_t orientation);
    void clear(uint16_t color);
    void setPrintColor(uint16_t color, uint16_t background);
    void setPrintSpacing(box_size_t spacing);
    void fillRectangle(point_t origin, box_size_t size, uint16_t color);
    void drawRectangle(point_t origin, box_size_t size, uint16_t color);
    void render(ScrollBuffer &sb, point_t origin, box_size_t size);
    void scrollArea(int y, int height);
    void scroll(int offset);
};

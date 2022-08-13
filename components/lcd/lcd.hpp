#pragma once

#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"

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

struct print_control_t
{
    int x;
    int y;
    int indent;
    int column_width;
    int line_height;
    uint16_t color;
    uint16_t background;
};

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

class Lcd
{
    lcd_orientation_t orientation;
    int height;
    int width;
    int xOffset;
    int yOffset;
    spi_device_handle_t device;
    gpio_num_t dcPin;
    void send(const void *, size_t, bool);
    void addressSetX(int, int);
    void addressSetY(int, int);
    void setWriteFrame(int, int, int, int);
    void printNormalChar(struct print_control_t *, char);
    void printControlChar(struct print_control_t *, char);
    void cursorAdvance(struct print_control_t *);
public:
    void init(lcd_init_config_t config, lcd_orientation_t orientation = LANDSCAPE);
    void displayOn(void);
    void brightness(int);
    int getHeight(void);
    int getWidth(void);
    void setOrientation(lcd_orientation_t orientation);
    void clear(uint16_t color);
    void printChar(struct print_control_t* handle, char c);
    void print(struct print_control_t* handle, const char* str);
    void fillRectangle(int x, int y, int width, int height, uint16_t color);
    void drawRectangle(int x, int y, int width, int height, uint16_t color);
    void scrollArea(int y, int height);
    void scroll(int offset);
};

#pragma once

#include <stdlib.h>

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

void lcd_init(void);

void lcd_display_on(void);

void lcd_brightness(int);

#define LCD_HEIGHT lcd_get_height()
int lcd_get_height(void);
#define LCD_WIDTH lcd_get_width()
int lcd_get_width(void);

void lcd_landscape(int up_down);

void lcd_portrait();

void lcd_clear(uint16_t color);

void lcd_print_char(struct print_control_t * handle, char c);

void lcd_print(struct print_control_t * handle, char *str);

void lcd_fill_rectangle(int x, int y, int width, int height, uint16_t color );

void lcd_draw_rectangle(int x, int y, int width, int height, uint16_t color );

void lcd_scroll_area(int y, int height);

void lcd_scroll(int offset);
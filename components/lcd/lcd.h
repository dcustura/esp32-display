#include <stdlib.h>

#define LCD_WIDTH 240
#define LCD_HEIGHT 135

#define rgb16(r, g, b) ((uint16_t)((0xf800 & ((r) << 8)) | \
                                   (0x07e0 & ((g) << 3)) | \
                                   (0x001f & ((b) >> 3))))

#define color(hex) ((uint16_t)((((hex) >> 8) & 0xf800) | \
                               (((hex) >> 5) & 0x07e0) | \
                               (((hex) >> 3)& 0x001f)))

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

void lcd_clear(uint16_t color);

void lcd_print_char(struct print_control_t * handle, char c);

void lcd_print(struct print_control_t * handle, char *str);

void lcd_draw_h_line(int x_start, int x_end, int y, uint16_t color);

void lcd_draw_v_line(int x, int y_start, int y_end, uint16_t color);

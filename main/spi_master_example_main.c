#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
//#include "driver/gpio.h"
#include "lcd.h"

#define delay(milliseconds) vTaskDelay((milliseconds) / portTICK_RATE_MS)

void app_main(void)
{
    lcd_init();
    lcd_clear(0);
    lcd_display_on();

    // lcd_draw_h_line(device, 0, LCD_WIDTH - 1, LCD_HEIGHT / 2, 0xffff);
    // lcd_draw_v_line(device, LCD_WIDTH / 2, 0, LCD_HEIGHT - 1, 0xffff);

    // crosshair(device, LCD_WIDTH / 2, LCD_HEIGHT / 2, 20);
    // crosshair(device, 0, 0, 20);
    // crosshair(device, LCD_WIDTH - 1, 0, 20);
    // crosshair(device, 0, LCD_HEIGHT - 1, 20);
    // crosshair(device, LCD_WIDTH - 1, LCD_HEIGHT - 1, 20);

    struct print_control_t pc = {
        .x = 0,
        .y = 0,
        //.indent = 0,
        .column_width = 12,
        .line_height = 17,
        .color = color(0x808080),
        .background = color(0x000000),
        .overstrike_color = color(0x404040),
    };

    lcd_print(&pc, "The quick brown fox jumps over the lazy dog! ");
    pc.color = color(0x000000);
    pc.overstrike_color = color(0xff0000);
    lcd_print(&pc, "The Quick Brown Fox Jumps Over The Lazy Dog? ");

    for (;;)
        delay(1000);
}

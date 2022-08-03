#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "lcd.h"

#define delay(milliseconds) vTaskDelay((milliseconds) / portTICK_RATE_MS)

#define rnd ((uint32_t)rand())

void my_task(void *ignore)
{
    lcd_init();
    lcd_clear(WHITE);
    lcd_display_on();

    // while (1) {
    //     int x = abs(rand()) % LCD_WIDTH;
    //     int y = abs(rand()) % LCD_HEIGHT;
    //     int width = 1 + (abs(rand()) % (LCD_WIDTH - x));
    //     int height = 1 + (abs(rand()) % (LCD_HEIGHT - y));
    //     uint16_t color = rand() % 0xffff;
    //     lcd_draw_rectangle(x, y, width, height, color);
    // }

    struct print_control_t pc = {
        .x = 0,
        .y = 0,
        //.indent = 0,
        .column_width = 13,
        .line_height = 20,
        .color = BLACK,
        .background = WHITE,
    };

    lcd_portrait();
    lcd_print(&pc, "The quick brown fox jumps over the lazy dog!\n\n");
    pc.color = GRAY;
    lcd_print(&pc, "The Quick Brown Fox Jumps Over The Lazy Dog?\n");

    //lcd_portrait();
    //for (;;) delay(1000);

    lcd_scroll_area(0, 240);
    for (;;)
    {
        for (int i = 0; i < 240; i += 1)
        {
            lcd_scroll(i);
            delay(20);
            lcd_brightness(i);
        }
    }
}

void app_main(void)
{
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_35, GPIO_MODE_INPUT);
    xTaskCreate(my_task, "my task", 2048, NULL, 5, NULL);
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "lcd.hpp"

#define delay(milliseconds) vTaskDelay((milliseconds) / portTICK_RATE_MS)

#define rnd ((uint32_t)rand())

void my_task(void* ignore)
{
    Lcd lcd;

#define LCD_HOST HSPI_HOST

    // hardware wiring:
#define PIN_NUM_MOSI 19
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 5
#define PIN_NUM_DC 16
#define PIN_NUM_RST 23
#define PIN_NUM_BCKL 4

    lcd.init((lcd_init_config_t) {
        .spi_host = HSPI_HOST,
            .pin_spi_mosi = GPIO_NUM_19,
            .pin_spi_clock = GPIO_NUM_18,
            .pin_spi_cs = GPIO_NUM_5,
            .pin_dc = GPIO_NUM_16,
            .pin_reset = GPIO_NUM_23,
            .pin_backlight = GPIO_NUM_4
    }, LANDSCAPE);

    lcd.clear(WHITE);
    lcd.displayOn();

    int lcdWidth = lcd.getWidth();
    int lcdHeight = lcd.getHeight();

    while (1) {
        int x = abs(rand()) % lcdWidth;
        int y = abs(rand()) % lcdHeight;
        int width = 1 + (abs(rand()) % (lcdWidth - x));
        int height = 1 + (abs(rand()) % (lcdHeight - y));
        uint16_t color = rand() % 0xffff;
        lcd.drawRectangle(x, y, width, height, color);
    }

    struct print_control_t pc = {
        .x = 0,
        .y = 0,
        //.indent = 0,
        .column_width = 13,
        .line_height = 20,
        .color = BLACK,
        .background = WHITE,
    };

    lcd.print(&pc, "The quick brown fox jumps over the lazy dog!\n\n");
    pc.color = GRAY;
    lcd.print(&pc, "The Quick Brown Fox Jumps Over The Lazy Dog?\n");

    // lcd_portrait();
    // for (;;) delay(1000);

    lcd.scrollArea(0, 240);
    for (;;)
    {
        for (int i = 0; i < 240; i += 1)
        {
            lcd.scroll(i);
            delay(20);
            //lcd.brightness(i);
        }
    }
}

extern "C" void app_main(void)
{
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_35, GPIO_MODE_INPUT);
    xTaskCreate(my_task, "my task", 2048, NULL, 5, NULL);
}

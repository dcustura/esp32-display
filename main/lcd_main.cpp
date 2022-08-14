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

#define LCD_HOST HSPI_HOST

    // hardware wiring:
#define PIN_NUM_MOSI 19
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 5
#define PIN_NUM_DC 16
#define PIN_NUM_RST 23
#define PIN_NUM_BCKL 4

    Lcd lcd = Lcd((lcd_init_config_t) {
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

    lcd.fillRectangle({ 20, 20 }, { 200, 100 }, BLUE);

   // for (;;) delay(1000);

    int lcdWidth = lcd.getSize().width;
    int lcdHeight = lcd.getSize().height;

    for (int i = 0; i < 1000; i++) {
        point_t origin = { abs(rand()) % lcdWidth, abs(rand()) % lcdHeight };
        box_size_t size = { 1 + (abs(rand()) % (lcdWidth - origin.x)), 1 + (abs(rand()) % (lcdHeight - origin.y)) };
        uint16_t color = rand() % 0xffff;
        if (i % 2) {
            lcd.drawRectangle(origin, size, color);
        } else {
            lcd.fillRectangle(origin, size, color);
        }
    }

    std::string hello = "hello!";

    lcd.setPrintColor(RED, GRAY);
    lcd << "The quick brown fox jumps over the lazy dog! ";
    lcd.setPrintColor(BLUE, BLACK);
    lcd << "The Quick Brown Fox Jumps Over The Lazy Dog? " << hello << 123;

    // lcd_portrait();
    for (;;) delay(1000);

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

#include <string>
#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "lcd.hpp"

using namespace std;

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
    lcd.clear(BLACK);
    lcd.displayOn();

    lcd.setPrintColor(WHITE, BLACK);
    ScrollBuffer sb(15, 4);

    sb.setForegroundColor((color_t){16, 32, 16});
    // color_t color = (color_t){ 0, 0, 0 };
    for (int c = 0; c < 256; c++)
    {
        lcd.render(sb, ZERO_XY, { 240, 135 });
        // sb.setForegroundColor(color);
        // color.r++; color.g += 2; color.b++;
        sb << (char) c;
        delay(200);
    }
    for (;;) delay(1000);

    // for (int i = 0; i < 1000; i++) {
    //     point_t origin = { abs(rand()) % lcdWidth, abs(rand()) % lcdHeight };
    //     box_size_t size = { 1 + (abs(rand()) % (lcdWidth - origin.x)), 1 + (abs(rand()) % (lcdHeight - origin.y)) };
    //     uint16_t color = rand() % 0xffff;
    //     if (i % 2) {
    //         lcd.drawRectangle(origin, size, color);
    //     } else {
    //         lcd.fillRectangle(origin, size, color);
    //     }
    // }
}

extern "C" void app_main(void)
{
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_35, GPIO_MODE_INPUT);
    xTaskCreate(my_task, "my task", 2048, NULL, 5, NULL);
}

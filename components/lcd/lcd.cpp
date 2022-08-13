#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/sigmadelta.h"

#include "lcd.hpp"
#include "font_12x16.hpp"

#define delay(milliseconds) vTaskDelay((milliseconds) / portTICK_RATE_MS)

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

#define hi(a) ((uint8_t)((a) >> 8))
#define lo(a) ((uint8_t)((a) & 0xff))

#define lcd_cmd(cmd, data...) ({      \
    uint8_t buffer[] = {data};        \
    uint8_t command = cmd;            \
    send(&command, 1, false);         \
    send(buffer, ARRAY_SIZE(buffer)); \
})

#define cmd_address_set(command, start, end) lcd_cmd((command), hi(start), lo(start), hi(end), lo(end))

#define cmd_write_data() lcd_cmd(0x2C)

#define BUFFER_SIZE LCD_LONG_WIDTH
static uint16_t pixel_buffer[BUFFER_SIZE];

void Lcd::send(const void* data, size_t len, bool isData = true)
{
    if (len == 0)
    {
        return; // no need to send anything
    }
    spi_transaction_t t = {
        .length = len * 8, // Len is in bytes, transaction length is in bits.
        .user = (void*)(dcPin | (isData ? (1 << 31) : 0)) // D/C
    };
    if (len > 4)
    {
        t.tx_buffer = data;
    }
    else
    {
        memcpy(t.tx_data, data, len);
        t.flags = SPI_TRANS_USE_TXDATA;
    }
    esp_err_t ret = spi_device_polling_transmit(device, &t); // Transmit!
    assert(ret == ESP_OK);                                   // Should have had no issues.
}

// This function is called (in irq context!) just before a transmission starts. It will
// set the D/C line to the value indicated in the user field.
void IRAM_ATTR lcd_spi_pre_transfer_callback(spi_transaction_t* t)
{
    uint32_t userPayload = (uint32_t)t->user;
    gpio_set_level((gpio_num_t)(userPayload & ~(1 << 31)), userPayload & (1 << 31));
}

void Lcd::addressSetX(int xStart, int xEnd)
{
    cmd_address_set(0x2A, xStart + xOffset, xEnd + xOffset);
}

void Lcd::addressSetY(int yStart, int yEnd)
{
    cmd_address_set(0x2B, yStart + yOffset, yEnd + yOffset);
}

void Lcd::setWriteFrame(int x, int y, int width, int height)
{
    addressSetX(x, x + width - 1);
    addressSetY(y, y + height - 1);
}

void Lcd::brightness(int value)
{
    sigmadelta_set_duty(SIGMADELTA_CHANNEL_0, value - 128);
}

void set16(uint16_t* dst, uint16_t color, int count)
{
    uint16_t swapped_color = SPI_SWAP_DATA_TX(color, 16);
    for (int i = 0; i < count; i++)
    {
        dst[i] = swapped_color;
    }
}

void Lcd::fillRectangle(int x, int y, int width, int height, uint16_t color)
{
    int num_pixels = width * height;
    set16(pixel_buffer, color, min(BUFFER_SIZE, num_pixels));
    setWriteFrame(x, y, width, height);
    cmd_write_data();
    int buffer_times = num_pixels / BUFFER_SIZE;
    for (int i = 0; i < buffer_times; i++)
    {
        send(pixel_buffer, BUFFER_SIZE * 2);
    }
    send(pixel_buffer, (num_pixels % BUFFER_SIZE) * 2);
}

void Lcd::drawRectangle(int x, int y, int width, int height, uint16_t color)
{
    fillRectangle(x, y, width, 1, color);              // top line
    fillRectangle(x, y, 1, height, color);             // left line
    fillRectangle(x + width - 1, y, 1, height, color); // right line
    fillRectangle(x, y + height - 1, width, 1, color); // bottom line
}

#define FONT_WIDTH 12
#define FONT_HEIGHT 16

void Lcd::cursorAdvance(struct print_control_t* handle)
{
    int new_x = handle->x + handle->column_width;
    if (new_x + FONT_WIDTH < width)
    {
        handle->x = new_x;
    }
    else
    {
        handle->x = handle->indent;
        handle->y += handle->line_height;
    }
}

void Lcd::printNormalChar(struct print_control_t* handle, char c)
{
    const uint16_t* glyph = font_12x16[(int)(c - '!')];
    int x = handle->x;
    int y = handle->y;
    uint16_t swapped_color = SPI_SWAP_DATA_TX(handle->color, 16);
    uint16_t swapped_background = SPI_SWAP_DATA_TX(handle->background, 16);
    setWriteFrame(x, y, handle->column_width, handle->line_height);
    cmd_write_data();
    for (int row = 0; row < FONT_HEIGHT; row++)
    {
        for (int col = 0; col < handle->column_width; col++)
        {
            pixel_buffer[col] = swapped_background;
            if (col < FONT_WIDTH && ((glyph[row] << col) & 0x8000))
            {
                pixel_buffer[col] = swapped_color;
            }
        }
        send(pixel_buffer, handle->column_width * 2);
    }
    if (FONT_HEIGHT < handle->line_height)
    {
        set16(pixel_buffer, handle->background, handle->column_width);
    }
    for (int i = FONT_HEIGHT; i < handle->line_height; i++)
    {
        send(pixel_buffer, handle->column_width * 2);
    }
    cursorAdvance(handle);
}

void Lcd::printControlChar(struct print_control_t* h, char c)
{
    switch (c)
    {
    case '\n':
        h->x = h->indent;
        h->y += h->line_height;
        return;
    case '\r':
        h->x = h->indent;
        return;
    default:
        fillRectangle(h->x, h->y, h->column_width, h->line_height, h->background);
        cursorAdvance(h);
        return;
    }
}

void Lcd::printChar(struct print_control_t* handle, char c)
{
    if (c > ' ')
    {
        printNormalChar(handle, c);
    }
    else
    {
        printControlChar(handle, c);
    }
}

void Lcd::print(struct print_control_t* handle, const char* str)
{
    for (const char* ptr = str; *ptr != 0; ptr++)
    {
        printChar(handle, *ptr);
    }
}

void Lcd::clear(uint16_t color)
{
    fillRectangle(0, 0, width, height, color);
}

void Lcd::displayOn(void)
{
    lcd_cmd(0x29);
}

int Lcd::getHeight(void)
{
    return height;
}

int Lcd::getWidth(void)
{
    return width;
}

#define MY ((uint8_t) 0x80)
#define MX ((uint8_t) 0x40)
#define MV ((uint8_t) 0x20)

void Lcd::setOrientation(lcd_orientation_t orientation) {
    this->orientation = orientation;
    switch (orientation)
    {
    case LANDSCAPE:
    case ROTATED_LANDSCAPE:
        lcd_cmd(0x36, (uint8_t)(MV | (orientation == ROTATED_LANDSCAPE ? MY : MX)));
        width = LCD_LONG_WIDTH;
        height = LCD_SHORT_WIDTH;
        xOffset = 40;
        yOffset = 53;
        break;
    case PORTRAIT:
    case ROTATED_PORTRAIT:
        lcd_cmd(0x36, (uint8_t)(orientation == ROTATED_PORTRAIT ? MX | MY : 0));
        width = LCD_SHORT_WIDTH;
        height = LCD_LONG_WIDTH;
        xOffset = 53;
        yOffset = 40;
        break;
    }
}

void Lcd::scrollArea(int y, int height)
{
    int top_fixed = 40 + y;
    int bottom_fixed = 320 - top_fixed - height;
    lcd_cmd(0x33, hi(top_fixed), lo(top_fixed), hi(height), lo(height), hi(bottom_fixed), lo(bottom_fixed));
}

void Lcd::scroll(int offset)
{
    int line_no = 40 + offset;
    lcd_cmd(0x37, hi(line_no), lo(line_no));
}

void Lcd::init(lcd_init_config_t config, lcd_orientation_t orientation)
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .mosi_io_num = config.pin_spi_mosi,
        .miso_io_num = -1,
        .sclk_io_num = config.pin_spi_clock,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BUFFER_SIZE * 2,
    };
    spi_device_interface_config_t devcfg = {
        .mode = 0,                               // SPI mode 0
        .clock_speed_hz = SPI_MASTER_FREQ_20M,
        .spics_io_num = config.pin_spi_cs,              // CS pin
        .queue_size = 7,                         // We want to be able to queue 7 transactions at a time
        .pre_cb = lcd_spi_pre_transfer_callback, // Specify pre-transfer callback to handle D/C line
    };
    // Initialize the SPI bus
    ret = spi_bus_initialize(config.spi_host, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    dcPin = config.pin_dc;
    // Attach the LCD to the SPI bus
    ret = spi_bus_add_device(config.spi_host, &devcfg, &device);
    ESP_ERROR_CHECK(ret);

    // Initialize non-SPI GPIOs
    gpio_set_direction(config.pin_dc, GPIO_MODE_OUTPUT);
    gpio_set_direction(config.pin_reset, GPIO_MODE_OUTPUT);
    gpio_set_direction(config.pin_backlight, GPIO_MODE_OUTPUT);

    sigmadelta_config_t sigmadelta_cfg = {
        .channel = SIGMADELTA_CHANNEL_0,
        .sigmadelta_duty = 0,
        .sigmadelta_prescale = 80,
        .sigmadelta_gpio = (uint8_t)config.pin_backlight,
    };
    sigmadelta_config(&sigmadelta_cfg);

    // Reset the display
    gpio_set_level(config.pin_reset, 0);
    delay(100);
    gpio_set_level(config.pin_reset, 1);
    delay(100);

    setOrientation(orientation);
    /* Interface Pixel Format, 16bits/pixel for RGB/MCU interface */
    lcd_cmd(0x3A, 0x55);
    /* Display Inversion On */
    lcd_cmd(0x21);
    lcd_cmd(0x53, 0x04);
    /* Sleep Out */
    lcd_cmd(0x11);
    delay(100);

    /// Enable backlight
    //gpio_set_level(PIN_NUM_BCKL, 1);
    brightness(127);
}

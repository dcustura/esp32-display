#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "ascii_8x12.h"
#include "ascii_8x13.h"

#define LCD_HOST HSPI_HOST

#define PIN_NUM_MOSI 19
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 5

#define PIN_NUM_DC 16
#define PIN_NUM_RST 23
#define PIN_NUM_BCKL 4

#define LCD_X_OFFS 40
#define LCD_Y_OFFS 53
#define LCD_WIDTH 240
#define LCD_HEIGHT 135

#define LCD_MAX_X (LCD_WIDTH + LCD_X_OFFS - 1)
#define LCD_MAX_Y (LCD_HEIGHT + LCD_Y_OFFS - 1)

#define delay(milliseconds) vTaskDelay((milliseconds) / portTICK_RATE_MS)

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

#define hi(a) ((a) >> 8)
#define lo(a) ((a)&0xff)

#define rgb16(r, g, b) ((uint16_t)((0xf800 & ((r) << 8)) | \
                                   (0x07e0 & ((g) << 3)) | \
                                   (0x001f & ((b) >> 3))))

#define color(hex) ((uint16_t)((((hex) >> 8) & 0xf800) | \
                               (((hex) >> 5) & 0x07e0) | \
                               (((hex) >> 3)& 0x001f)))

#define lcd_cmd(dev, cmd, data...) ({             \
    uint8_t buffer[] = {data};                    \
    uint8_t command = cmd;                        \
    lcd_send(dev, &command, 1, 0);                \
    lcd_send(dev, buffer, ARRAY_SIZE(buffer), 1); \
})

#define lcd_data(dev, data) lcd_send((dev), (data), sizeof(data), 1)

#define lcd_cmd_address_set(device, command, start, end) lcd_cmd((device), (command), hi(start), lo(start), hi(end), lo(end))
#define lcd_x_address(device, xs, xe) lcd_cmd_address_set((device), 0x2A, (xs + LCD_X_OFFS), (xe + LCD_X_OFFS))
#define lcd_y_address(device, ys, ye) lcd_cmd_address_set((device), 0x2B, (ys + LCD_Y_OFFS), (ye + LCD_Y_OFFS))
#define lcd_set_write_frame(device, x, y, width, height) \
    lcd_x_address((device), (x), (x) + (width)-1);       \
    lcd_y_address((device), (y), (y) + (height)-1);

#define lcd_cmd_write_data(device) lcd_cmd((device), 0x2C)

void lcd_send(spi_device_handle_t device, const void *data, int len, int dc)
{
    if (len == 0)
    {
        return; // no need to send anything
    }
    spi_transaction_t t = {
        .length = len * 8, // Len is in bytes, transaction length is in bits.
        .user = (void *)dc // D/C
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
void IRAM_ATTR lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    gpio_set_level(PIN_NUM_DC, (uint32_t)t->user);
}

void set16(uint16_t *dst, uint16_t color, int count)
{
    uint16_t swapped_color = SPI_SWAP_DATA_TX(color, 16);
    for (int i = 0; i < count; i++)
    {
        dst[i] = swapped_color;
    }
}

static uint16_t pixel_buffer[LCD_WIDTH];

void lcd_draw_h_line(spi_device_handle_t device, int x_start, int x_end, int y, uint16_t color)
{
    int xs = min(x_start, x_end);
    int xe = max(x_start, x_end);
    int line_length = xe - xs + 1;
    set16(pixel_buffer, color, line_length);
    lcd_set_write_frame(device, xs, y, line_length, 1);
    lcd_cmd_write_data(device);
    lcd_send(device, pixel_buffer, line_length * 2, 1);
}

void lcd_draw_v_line(spi_device_handle_t device, int x, int y_start, int y_end, uint16_t color)
{
    int ys = min(y_start, y_end);
    int ye = max(y_start, y_end);
    int line_length = ye - ys + 1;
    set16(pixel_buffer, color, line_length);
    lcd_set_write_frame(device, x, ys, 1, line_length);
    lcd_cmd_write_data(device);
    lcd_send(device, pixel_buffer, line_length * 2, 1);
}

void crosshair(spi_device_handle_t device, int x, int y, int r)
{
    uint16_t color = 0xffff;
    lcd_draw_h_line(device, x - r, x + r, y, color);
    lcd_draw_v_line(device, x, y - r, y + r, color);
}

// Initialize the display
void lcd_init(spi_device_handle_t device)
{
    // Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    // Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    delay(100);
    gpio_set_level(PIN_NUM_RST, 1);
    delay(100);

    /* Memory Data Access Control, MX=MV=1, MY=ML=MH=0, RGB=0 */
    lcd_cmd(device, 0x36, 0x60);
    /* Interface Pixel Format, 16bits/pixel for RGB/MCU interface */
    lcd_cmd(device, 0x3A, 0x55);
    /* Display Inversion On */
    lcd_cmd(device, 0x21);
    /* Sleep Out */
    lcd_cmd(device, 0x11);
    delay(100);

    /// Enable backlight
    gpio_set_level(PIN_NUM_BCKL, 1);
}

struct print_control_t
{
    int x;
    int y;
    int indent;
    int column_width;
    int line_height;
    uint16_t color;
    uint16_t background;
    uint16_t overstrike_color;
    spi_device_handle_t device;
};

#define FONT_WIDTH 8
#define FONT_HEIGHT 13

void lcd_print_char(struct print_control_t * handle, char c)
{
    const uint8_t * glyph = ascii_8x13_font[(int)(c - ' ')];
    int x = handle->x;
    int y = handle->y;
    uint16_t swapped_color = SPI_SWAP_DATA_TX(handle->color, 16);
    uint16_t swapped_background = SPI_SWAP_DATA_TX(handle->background, 16);
    uint16_t swapped_overstrike = SPI_SWAP_DATA_TX(handle->overstrike_color, 16);
    spi_device_handle_t device = handle->device;
    lcd_set_write_frame(device, x, y, handle->column_width, handle->line_height);
    lcd_cmd_write_data(device);
    for (int row = 0; row < FONT_HEIGHT + 1; row++)
    {
        for (int col = 0; col < handle->column_width; col++)
        {
            pixel_buffer[col] = swapped_background;
            if (row > 0 && col < FONT_WIDTH && ((glyph[row - 1] >> (FONT_WIDTH - 1 - col)) & 1)) {
                pixel_buffer[col] = swapped_overstrike;
            }
            if (row < FONT_HEIGHT && col > 0 && col < FONT_WIDTH + 1 && ((glyph[row] >> (FONT_WIDTH - col)) & 1))
            {
                pixel_buffer[col] = swapped_overstrike;
            }
            if (row < FONT_HEIGHT && col < FONT_WIDTH && ((glyph[row] >> (FONT_WIDTH - 1 - col)) & 1))
            {
                pixel_buffer[col] = swapped_color;
            }
        }
        lcd_send(device, pixel_buffer, handle->column_width * 2, 1);
    }
    if (FONT_HEIGHT < handle->line_height)
    {
        set16(pixel_buffer, handle->background, handle->column_width);
    }
    for (int i = FONT_HEIGHT + 1; i < handle->line_height; i++)
    {
        lcd_send(device, pixel_buffer, handle->column_width * 2, 1);
    }
    int new_x = handle->x + handle->column_width;
    if (new_x + FONT_WIDTH < LCD_WIDTH)
    {
        handle->x = new_x;
    }
    else
    {
        handle->x = handle->indent;
        handle->y += handle->line_height;
    }
}

void lcd_print(struct print_control_t * handle, char *str)
{
    for (char *ptr = str; *ptr != 0; ptr++)
    {
        lcd_print_char(handle, *ptr);
    }
}

void lcd_clear(spi_device_handle_t device, uint16_t color)
{
    set16(pixel_buffer, color, LCD_WIDTH);
    lcd_set_write_frame(device, 0, 0, LCD_WIDTH, LCD_HEIGHT);
    lcd_cmd_write_data(device);
    for (int i = 0; i < LCD_HEIGHT; i++)
    {
        lcd_send(device, pixel_buffer, LCD_WIDTH * 2, 1);
    }
}

void app_main(void)
{
    esp_err_t ret;
    spi_device_handle_t device;
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        //.max_transfer_sz = 0,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_MASTER_FREQ_20M,
        .mode = 0,                               // SPI mode 0
        .spics_io_num = PIN_NUM_CS,              // CS pin
        .queue_size = 7,                         // We want to be able to queue 7 transactions at a time
        .pre_cb = lcd_spi_pre_transfer_callback, // Specify pre-transfer callback to handle D/C line
    };
    // Initialize the SPI bus
    ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    // Attach the LCD to the SPI bus
    ret = spi_bus_add_device(LCD_HOST, &devcfg, &device);
    ESP_ERROR_CHECK(ret);
    // Initialize the LCD
    lcd_init(device);

    lcd_clear(device, 0);

    /* Display On */
    lcd_cmd(device, 0x29);

    // lcd_draw_h_line(device, 0, LCD_WIDTH - 1, LCD_HEIGHT / 2, 0xffff);
    // lcd_draw_v_line(device, LCD_WIDTH / 2, 0, LCD_HEIGHT - 1, 0xffff);

    // crosshair(device, LCD_WIDTH / 2, LCD_HEIGHT / 2, 20);
    // crosshair(device, 0, 0, 20);
    // crosshair(device, LCD_WIDTH - 1, 0, 20);
    // crosshair(device, 0, LCD_HEIGHT - 1, 20);
    // crosshair(device, LCD_WIDTH - 1, LCD_HEIGHT - 1, 20);

    struct print_control_t pc = {
        .device = device,
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

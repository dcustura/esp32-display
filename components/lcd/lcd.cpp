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


#define FONT_WIDTH 12
#define FONT_HEIGHT 16

enum LcdCommand {
    CMD_WRITE_DATA = 0x2C,
    CMD_SET_X_ADDRESS = 0x2A,
    CMD_SET_Y_ADDRESS = 0x2B,
    CMD_DISPLAY_ON = 0x29,
    CMD_SET_ADDRESS_MODE = 0x36,
    CMD_SET_SCROLL_AREA = 0x33,
    CMD_SCROLL = 0x37,
    CMD_SET_PIXEL_FORMAT = 0x3A,
    CMD_INVERSION_ON = 0x21,
    CMD_SLEEP_OFF = 0x11
};

point_t operator+(point_t point, box_size_t size)
{
    return (point_t) { point.x + size.width, point.y + size.height };
}

point_t operator-(point_t point, int amount)
{
    return (point_t) { point.x - amount, point.y - amount};
}

struct Lcd::PrintControl {
    point_t position;
    point_t minPosition;
    point_t maxPosition;
    box_size_t spacing = { 14, 20 };
    uint16_t color = WHITE, background = BLACK;

    void setFrame(point_t origin, box_size_t size)
    {
        position = origin;
        minPosition = origin;
        maxPosition = origin + size - 1;
    }

    void setXY(point_t newPosition) {
        position = newPosition;
    }

    void setColor(uint16_t newColor, uint16_t newBackground)
    {
        color = newColor;
        background = newBackground;
    }

    void setSpacing(box_size_t newSpacing)
    {
        spacing = newSpacing;
    }

    void cursorAdvanceRow()
    {
        position.x = minPosition.x;
        position.y += spacing.height;
        if (position.y + FONT_HEIGHT > maxPosition.y)
        {
            position.y = minPosition.y;
        }
    }

    void cursorAdvance()
    {
        position.x += spacing.width;
        if (position.x + FONT_WIDTH > maxPosition.x)
        {
            cursorAdvanceRow();
        }
    }
};

inline void delay(int milliseconds)
{
    vTaskDelay(milliseconds / portTICK_RATE_MS);
}

template <typename T> inline T min(T a, T b)
{
    return a < b ? a : b;
}

inline uint8_t hi(uint16_t word)
{
    return word >> 8;
}

inline uint8_t lo(uint16_t word)
{
    return word & 0xff;
}

#define BUFFER_SIZE LCD_LONG_WIDTH
static uint16_t pixel_buffer[BUFFER_SIZE];

void lcd_send(spi_device_handle_t device, gpio_num_t dcPin, const void* data, size_t len, bool isData = true)
{
    if (len == 0)
    {
        return;
    }
    spi_transaction_t t = {
        .length = len * 8, // transaction length is in bits.
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
    esp_err_t ret = spi_device_polling_transmit(device, &t);
    assert(ret == ESP_OK);
}


#define cmd Command(device, dcPin)

#define MAX_COMMAND_BUFFER_SIZE 8

class Command
{
    spi_device_handle_t device;
    gpio_num_t dcPin;
    uint8_t buffer[MAX_COMMAND_BUFFER_SIZE];
    int dataSize;
    void flush()
    {
        if (dataSize > 0)
        {
            lcd_send(device, dcPin, buffer, dataSize);
            dataSize = 0;
        }
    }
public:
    Command(spi_device_handle_t device, gpio_num_t dcPin) : device(device), dcPin(dcPin), dataSize(0) {}

    ~Command()
    {
        flush();
    }

    Command& operator^(LcdCommand command)
    {
        flush();
        lcd_send(device, dcPin, &command, 1, false);
        return *this;
    }

    Command& operator,(uint8_t dataByte)
    {
        if (dataSize == MAX_COMMAND_BUFFER_SIZE)
        {
            flush();
        }
        buffer[dataSize++] = dataByte;
        return *this;
    }

    Command& operator,(uint16_t dataWord)
    {
        return *this, hi(dataWord), lo(dataWord);
    }
};

void Lcd::send(const void* data, size_t len, bool isData = true)
{
    lcd_send(device, dcPin, data, len, isData);
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
    cmd^ CMD_SET_X_ADDRESS, (uint16_t)(xStart + xOffset), (uint16_t)(xEnd + xOffset);
}

void Lcd::addressSetY(int yStart, int yEnd)
{
    cmd^ CMD_SET_Y_ADDRESS, (uint16_t)(yStart + yOffset), (uint16_t)(yEnd + yOffset);
}

void Lcd::setWriteFrame(point_t origin, box_size_t size)
{
    addressSetX(origin.x, origin.x + size.width - 1);
    addressSetY(origin.y, origin.y + size.height - 1);
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

void Lcd::fillRectangle(point_t origin, box_size_t size, uint16_t color)
{
    int num_pixels = size.width * size.height;
    set16(pixel_buffer, color, min(BUFFER_SIZE, num_pixels));
    setWriteFrame(origin, size);
    cmd^ CMD_WRITE_DATA;
    int buffer_times = num_pixels / BUFFER_SIZE;
    for (int i = 0; i < buffer_times; i++)
    {
        send(pixel_buffer, BUFFER_SIZE * 2);
    }
    send(pixel_buffer, (num_pixels % BUFFER_SIZE) * 2);
}

void Lcd::drawRectangle(point_t origin, box_size_t size, uint16_t color)
{
    box_size_t verticalLineSize = (box_size_t){ 1, size.height };
    box_size_t horizontalLineSize = (box_size_t){ size.width, 1 };
    fillRectangle(origin, horizontalLineSize, color);
    fillRectangle(origin, verticalLineSize, color);
    fillRectangle(origin + horizontalLineSize - 1, verticalLineSize , color);
    fillRectangle(origin + verticalLineSize - 1, horizontalLineSize, color);
}

void Lcd::setPrintFrame(point_t origin, box_size_t size)
{
    printControl->setFrame(origin, size);
}

void Lcd::setPrintXY(point_t position)
{
    printControl->setXY(position);
}

void Lcd::setPrintColor(uint16_t color, uint16_t background)
{
    printControl->setColor(color, background);
}

void Lcd::setPrintSpacing(box_size_t spacing)
{
    printControl->setSpacing(spacing);
}

void Lcd::printNormalChar(char c)
{
    const uint16_t* glyph = font_12x16[(int)(c - '!')];
    uint16_t swapped_color = SPI_SWAP_DATA_TX(printControl->color, 16);
    uint16_t swapped_background = SPI_SWAP_DATA_TX(printControl->background, 16);
    setWriteFrame(printControl->position, printControl->spacing);
    cmd^ CMD_WRITE_DATA;
    for (int row = 0; row < FONT_HEIGHT; row++)
    {
        for (int col = 0; col < printControl->spacing.width; col++)
        {
            pixel_buffer[col] = swapped_background;
            if (col < FONT_WIDTH && ((glyph[row] << col) & 0x8000))
            {
                pixel_buffer[col] = swapped_color;
            }
        }
        send(pixel_buffer, printControl->spacing.width * 2);
    }
    if (FONT_HEIGHT < printControl->spacing.height)
    {
        set16(pixel_buffer, printControl->background, printControl->spacing.width);
    }
    for (int i = FONT_HEIGHT; i < printControl->spacing.height; i++)
    {
        send(pixel_buffer, printControl->spacing.width * 2);
    }
    printControl->cursorAdvance();
}

void Lcd::printControlChar(char c)
{
    switch (c)
    {
    case '\n':
        printControl->cursorAdvanceRow();
        return;
    case '\r':
        printControl->position.x = printControl->minPosition.x;
        return;
    default:
        fillRectangle(printControl->position, printControl->spacing, printControl->background);
        printControl->cursorAdvance();
        return;
    }
}

void Lcd::printChar(char c)
{
    if (c > ' ')
    {
        printNormalChar(c);
    }
    else
    {
        printControlChar(c);
    }
}

void Lcd::print(const char* str)
{
    for (const char* ptr = str; *ptr != 0; ptr++)
    {
        printChar(*ptr);
    }
}

Lcd& Lcd::operator<<(const char* str)
{
    print(str);
    return *this;
}

Lcd& Lcd::operator<<(char c)
{
    printChar(c);
    return *this;
}

Lcd& Lcd::operator<<(std::string str)
{
    for (char c : str)
    {
        printChar(c);
    }
    return *this;
}

Lcd& Lcd::operator<<(int num)
{
    return *this << std::to_string(num);
}

void Lcd::clear(uint16_t color)
{
    fillRectangle(ZERO_XY, size, color);
}

void Lcd::displayOn(void)
{
    cmd^ CMD_DISPLAY_ON;
}

box_size_t Lcd::getSize(void)
{
    return size;
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
        cmd^ CMD_SET_ADDRESS_MODE, (uint8_t)(MV | (orientation == ROTATED_LANDSCAPE ? MY : MX));
        size = { LCD_LONG_WIDTH, LCD_SHORT_WIDTH };
        xOffset = 40;
        yOffset = 53;
        break;
    case PORTRAIT:
    case ROTATED_PORTRAIT:
        cmd^ CMD_SET_ADDRESS_MODE, (uint8_t)(orientation == ROTATED_PORTRAIT ? MX | MY : 0);
        size = { LCD_SHORT_WIDTH, LCD_LONG_WIDTH };
        xOffset = 53;
        yOffset = 40;
        break;
    }
}

void Lcd::scrollArea(int y, int height)
{
    int top_fixed = 40 + y;
    int bottom_fixed = 320 - top_fixed - height;
    cmd^ CMD_SET_SCROLL_AREA, (uint16_t)top_fixed, (uint16_t)height, (uint16_t)bottom_fixed;
}

void Lcd::scroll(int offset)
{
    cmd^ CMD_SCROLL, (uint16_t)(40 + offset);
}

Lcd::Lcd(lcd_init_config_t config, lcd_orientation_t orientation)
    : printControl(new PrintControl)
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
        .spics_io_num = config.pin_spi_cs,       // CS pin
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
    printControl->setFrame(ZERO_XY, size);
    /* Interface Pixel Format, 16bits/pixel for RGB/MCU interface */
    cmd^ CMD_SET_PIXEL_FORMAT, (uint8_t)0x55;
    /* Display Inversion On */
    cmd^ CMD_INVERSION_ON;
    /* Sleep Out */
    cmd^ CMD_SLEEP_OFF;
    delay(100);

    /// Enable backlight
    brightness(127);
}

Lcd::~Lcd() {}

#pragma once
#include <string>

typedef unsigned short int uint16_t;

typedef struct color
{
    uint16_t r : 5;
    uint16_t g : 6;
    uint16_t b : 5;
    uint16_t to16()
    {
        return *((uint16_t *)this);
    }
    bool operator!=(struct color other)
    {
        return this->to16() != other.to16();
    }
} color_t;

typedef struct
{
    char c;
    color_t foreground;
    color_t background;
} color_char_t;

typedef struct
{
    unsigned int fromRow;
    unsigned int toRow;
    unsigned int fromCol;
    unsigned int toCol;
} area_t;

class ScrollBuffer
{
    char *buffer;
    color_t *foregroundColors, *backgroundColors;
    unsigned int bufferColumns, bufferRows;
    unsigned int crtRow, crtZeroRow, crtColumn;
    color_t crtForeground, crtBackground;
    area_t dirtyArea;

    void checkColumn();
    void advanceRow();
    void setDirty(unsigned int);

public:
    ScrollBuffer(
        unsigned int bufferColumns, unsigned int bufferRows,
        color_t foregroundColor = (color_t) { 31, 63, 31 },
        color_t backgroundColor = (color_t) { 0, 0, 0 }
    );
    ~ScrollBuffer();
    void clear();
    void setForegroundColor(color_t color);
    void setBackgroundColor(color_t color);
    ScrollBuffer &operator<<(const char c);
    ScrollBuffer &operator<<(const char *str);
    ScrollBuffer &operator<<(std::string str);
    int getRows();
    char getCharAt(unsigned int row, unsigned int column);
    color_char_t getColorCharAt(unsigned int row, unsigned int column);
    area_t getDirtyArea();
    void resetDirty();
    void debug();
};

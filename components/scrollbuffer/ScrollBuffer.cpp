#include <iostream>
#include <string>
#include <cstring>
#include <algorithm>
#include "ScrollBuffer.hpp"

using namespace std;

ScrollBuffer::ScrollBuffer(
    unsigned int bufferColumns, unsigned int bufferRows,
    color_t foregroundColor, color_t backgroundColor)
    : bufferColumns(bufferColumns), bufferRows(bufferRows),
    crtRow(0), crtZeroRow(0), crtColumn(0),
    crtForeground(foregroundColor), crtBackground(backgroundColor)
{
    auto bufferSize = bufferColumns * bufferRows;
    buffer = new char[bufferSize];
    foregroundColors = new color_t[bufferSize];
    backgroundColors = new color_t[bufferSize];
    clear();
}

ScrollBuffer::~ScrollBuffer()
{
    delete buffer;
    delete foregroundColors;
    delete backgroundColors;
}

void ScrollBuffer::clear()
{
    memset(buffer, ' ', bufferColumns * bufferRows);
    for (int i = 0; i < bufferColumns * bufferRows; i++)
    {
        foregroundColors[i] = crtForeground;
        backgroundColors[i] = crtBackground;
    }
    crtRow = crtZeroRow = crtColumn = 0;
    dirtyArea = (area_t){
        .fromRow = 0,
        .toRow = bufferRows,
        .fromCol = 0,
        .toCol = bufferColumns
    };
}

void ScrollBuffer::setForegroundColor(color_t color)
{
    crtForeground = color;
}

void ScrollBuffer::setBackgroundColor(color_t color)
{
    crtBackground = color;
}

int ScrollBuffer::getRows()
{
    return (bufferRows + crtRow - crtZeroRow) % bufferRows + 1;
}

area_t ScrollBuffer::getDirtyArea()
{
    return dirtyArea;
}

char ScrollBuffer::getCharAt(unsigned int row, unsigned int column)
{
    int index = ((row + crtZeroRow) % bufferRows) * bufferColumns + column;
    return buffer[index];
}

color_char_t ScrollBuffer::getColorCharAt(unsigned int row, unsigned int column)
{
    int index = ((row + crtZeroRow) % bufferRows) * bufferColumns + column;
    return (color_char_t) { buffer[index], foregroundColors[index], backgroundColors[index] };
}

template<typename T> void incMod(T& value, T mod)
{
    value = (value + 1) % mod;
}

void ScrollBuffer::setDirty(unsigned int toCol)
{
    unsigned int row = (bufferRows - crtZeroRow + crtRow) % bufferRows;
    bool previouslyClean = dirtyArea.fromRow == dirtyArea.toRow && dirtyArea.fromCol == dirtyArea.toCol;
    dirtyArea = previouslyClean
        ? (area_t) {
        .fromRow = row,
            .toRow = row + 1,
            .fromCol = crtColumn,
            .toCol = toCol
    }
    : (area_t) {
        .fromRow = min(dirtyArea.fromRow, row),
            .toRow = max(dirtyArea.toRow, row + 1),
            .fromCol = min(dirtyArea.fromCol, crtColumn),
            .toCol = max(dirtyArea.toCol, toCol)
    };
}

void ScrollBuffer::advanceRow()
{
    crtColumn = 0;
    incMod(crtRow, bufferRows);
    setDirty(crtColumn);
    if (crtRow == crtZeroRow)
    {
        incMod(crtZeroRow, bufferRows);
        memset(buffer + crtRow * bufferColumns, ' ', bufferColumns);
        for (int i = 0; i < bufferColumns; i++)
        {
            foregroundColors[crtRow * bufferColumns + i] = crtForeground;
            backgroundColors[crtRow * bufferColumns + i] = crtBackground;
        }
        dirtyArea = (area_t){
            .fromRow = 0,
            .toRow = bufferRows,
            .fromCol = 0,
            .toCol = bufferColumns
        };
    }
}

void ScrollBuffer::checkColumn()
{
    if (crtColumn >= bufferColumns)
    {
        advanceRow();
    }
}

ScrollBuffer& ScrollBuffer::operator<<(const char c)
{
    checkColumn();
    int index = crtRow * bufferColumns + crtColumn;
    buffer[index] = c;
    foregroundColors[index] = crtForeground;
    backgroundColors[index] = crtBackground;
    setDirty(crtColumn + 1);
    crtColumn++;
    return *this;
}

ScrollBuffer& ScrollBuffer::operator<<(const char* str)
{
    for (const char* p = str; *p; p++)
    {
        *this << *p;
    }
    return *this;
}

ScrollBuffer& ScrollBuffer::operator<<(std::string str)
{
    for (char c : str)
    {
        *this << c;
    }
    return *this;
}

void ScrollBuffer::resetDirty()
{
    dirtyArea.fromRow = dirtyArea.toRow = (bufferRows + crtRow - crtZeroRow) % bufferRows;
    dirtyArea.fromCol = dirtyArea.toCol = crtColumn;
}

void outFill(char c, int bufferColumns)
{
    for (int i = 0; i < bufferColumns; i++)
    {
        cout << c;
    }
    cout << endl;
}

void ScrollBuffer::debug()
{
    cout << ' ';
    outFill('-', bufferColumns);
    char* p = buffer;
    for (int i = 0; i < bufferRows; i++)
    {
        cout << (crtRow == i ? '>' : (crtZeroRow == i ? '-' : '|'));
        for (int j = 0; j < bufferColumns; j++)
        {
            char c = *p;
            cout << (c >= ' ' ? c : '.');
            p++;
        }
        cout << '|' << endl;
    }
    cout << ' ';
    outFill('-', bufferColumns);
}

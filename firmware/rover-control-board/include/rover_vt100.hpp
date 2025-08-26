#pragma once

#include <stdint.h>

#define VT100_COLOR_BLACK   0
#define VT100_COLOR_RED     1
#define VT100_COLOR_GREEN   2
#define VT100_COLOR_YELLOW  3
#define VT100_COLOR_BLUE    4
#define VT100_COLOR_MAGENTA 5
#define VT100_COLOR_CYAN    6
#define VT100_COLOR_WHITE   7
#define VT100_COLOR_DEFAULT 9

void vt100ResetTerminal();

void vt100SetBackgroundColor(uint8_t color);
void vt100SetForegroundColor(uint8_t color);
void vt100SetColors(uint8_t foreground, uint8_t background);
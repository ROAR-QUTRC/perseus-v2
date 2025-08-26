#include "rover_vt100.hpp"

#include <stdio.h>

void vt100ResetTerminal()
{
    printf("\033[0m\r\n");
}

void vt100SetBackgroundColor(uint8_t color)
{
    printf("\033[3%dm", color);
}
void vt100SetForegroundColor(uint8_t color)
{
    printf("\033[3%dm", color);
}
void vt100SetColors(uint8_t fgCol, uint8_t bgCol)
{
    printf("\033[3%d;4%dm", fgCol, bgCol);
}
#include "rover_log.hpp"

// C lib
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// STL
#include <mutex>

// drivers
#include <driver/uart.h>

// core
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

std::mutex logMutex;

void logPrint(const char* prefix, const char* tag, const uint8_t color, const char* fmt, ...)
{
    // acquire mutex lock for thread safety
    std::lock_guard<std::mutex> lock(logMutex);
    vt100SetForegroundColor(color);
    fflush(stdout);
    // printf (and stdout) go to UART_0
    printf("[%s:%s](%lld) ", prefix, tag, esp_timer_get_time() / 1000);
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    vt100ResetTerminal();
}
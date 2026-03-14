#include "rover_core.hpp"

// core
#include <esp_adc/adc_cali.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

// standard libraries
#include <cstdio>
#include <rover_io.hpp>
#include <rover_thread.hpp>

static bool initialised = false;

void core_init()
{
    // if your code makes the ESP enter a boot loop,
    // give a window before running it to allow you to re-program the board.
    DELAY_MS(3000);
    // TODO: we need to track re-initialisation since we register an idle task for computing CPU usage
    if (initialised)
    {
        printf("Core initialisation already completed!");
        return;
    }
    DELAY_MS(100);  // allow UART TX buffer to flush
    // UART_0 is default initialised at 115200 baud for stdin/out/err, but we need to explicitly install the driver for normal uart i/o
    // also prevents buffer overflows
    initialize_uart(UART_NUM_0, 115200);

    printf("Begin core initialisation");

    initialize_nvs();

    gpio_install_isr_service(0);  // enable per-pin ISRs on `*this core*

    initialised = true;
    printf("End core initialisation");
}

int64_t core_get_uptime()
{
    // this function returns time since boot in us, we want ms
    return (esp_timer_get_time() / 1000);
}
uint8_t core_get_usage()
{
    return 100;  // TODO: implement using idle task - surprisingly complicated
}
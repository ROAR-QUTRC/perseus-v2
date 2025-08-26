#include "rover_core.hpp"

// core
#include <esp_adc/adc_cali.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

// standard libraries
#include <cstdio>

// arc libraries
// #include <rover_adc.hpp>
#include <canlib.hpp>
#include <rover_io.hpp>
#include <rover_log.hpp>
#include <rover_thread.hpp>

static bool initialised = false;

void coreInit()
{
    // if your code makes the ESP enter a boot loop,
    // give a window before running it to allow you to re-program the board.
    DELAY_MS(3000);
    // TODO: we need to track re-initialisation since we register an idle task for computing CPU usage
    if (initialised)
    {
        CORE_ERROR("Core initialisation already completed!");
        return;
    }
    DELAY_MS(100);  // allow UART TX buffer to flush
    // UART_0 is default initialised at 115200 baud for stdin/out/err, but we need to explicitly install the driver for normal uart i/o
    // also prevents buffer overflows
    ioInitUart(UART_NUM_0, 115200);

    CORE_INFO("Begin core initialisation");

    ioInitNvs();
    ioInitFiles();
    // initWifi();
    // initServer();

    gpio_install_isr_service(0);  // enable per-pin ISRs on `*this core*

    initialised = true;
    CORE_DEBUG("End core initialisation");
}

void coreRestart()
{
    CORE_WARN("Shutting down services");

    ioStopFiles();

    canlibStop();

    CORE_WARN("Restarting");
    fflush(stdout);

    DELAY_MS(10);  // allow time for UART buffer to flush
    esp_restart();
}

int64_t coreGetUptime()
{
    // this function returns time since boot in us, we want ms
    return (esp_timer_get_time() / 1000);
}
uint8_t coreGetUsage()
{
    return 100;  // TODO: implement using idle task - surprisingly complicated
}
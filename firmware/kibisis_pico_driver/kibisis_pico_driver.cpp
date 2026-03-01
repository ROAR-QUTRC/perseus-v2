#include <cstdio>

#include "config.hpp"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "kibisis_encoder.hpp"
#include "kibisis_pwm_converter.hpp"
#include "pico/stdlib.h"

// ─────────────────────────────────────────────────────────────────────────────
// kibisis_pico_driver.cpp
//
// Initialises all hardware modules and runs the main loop.
// Each module owns its own init and per-loop update function.
// This file contains no hardware logic – it only orchestrates.
// ─────────────────────────────────────────────────────────────────────────────

int main()
{
    stdio_init_all();

    // ── Watchdog ──────────────────────────────────────────────────────────────
    if (watchdog_caused_reboot())
    {
        printf("Rebooted by watchdog!\n");
        // Add any post-watchdog-reboot recovery logic here
    }

    watchdog_enable(WATCHDOG_TIMEOUT_MS, WATCHDOG_PAUSE_DEBUG);

    // ── I2C peripheral (shared – init once here, before any module that uses it)
    i2c_init(I2C_INSTANCE, I2C_BAUDRATE);

    // ── Module init ───────────────────────────────────────────────────────────
    kibisis::pwm_converter_init();
    kibisis::encoder_init();

    // ── Main loop ─────────────────────────────────────────────────────────────
    while (true)
    {
        watchdog_update();

        kibisis::encoder_update();  // drains PIO FIFOs + calls set_encoder()
        kibisis::pwm_converter_apply_updates();

        sleep_us(500);
    }
}

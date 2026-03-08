<<<<<<< HEAD
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
=======
#include <stdio.h>
#include "pico/stdlib.h"
#include "i2c_slave.hpp"
#include "drive_motors.hpp"
#include "space_resources_motor.hpp"

static constexpr uint LED_PIN = PICO_DEFAULT_LED_PIN;

int main() {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);  // solid on - program is running

    I2CSlave slave;
    slave.init();

    DriveMotorController motors;
    motors.init();

    SpaceResourcesMotor space_motor;
    space_motor.init();

    printf("Kibisis Pico Driver ready. I2C slave @ 0x%02X\n", SLAVE_ADDR);

    // Placeholder encoder counts for testing
    // These will be replaced by real encoder reads in a later step
    int32_t enc_a = 0;
    int32_t enc_b = 0;

    while (true) {
        // Write latest values into shadow buffer
        slave.setEncoderA(enc_a++);
        slave.setEncoderB(enc_b++);
        slave.setStatus(0x01);

        // Atomically commit shadow -> live so IRQ always reads a consistent snapshot
        slave.commitRegisters();

        // Fetch what the Pi 5 has written and apply to motors
        int8_t motor_a     = slave.getMotorASpeed();
        int8_t motor_b     = slave.getMotorBSpeed();
        int8_t space_speed = slave.getSpaceMotorSpeed();

        motors.setMotorA(motor_a);
        motors.setMotorB(motor_b);
        space_motor.setSpeed(space_speed);

        printf("Motor A: %d  Motor B: %d  Space: %d  Enc A: %ld  Enc B: %ld\n",
               motor_a, motor_b, space_speed, enc_a, enc_b);

        sleep_ms(100);
>>>>>>> add936bf (feat(kibisis_firmware): pico i2c slave implementation)
    }
}

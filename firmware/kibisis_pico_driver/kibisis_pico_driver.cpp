#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "i2c_slave.hpp"
#include "drive_motors.hpp"
#include "space_resources_motor.hpp"
#include "quadrature_encoders.hpp"
#include "moisture_sensor.hpp"

static constexpr uint LED_PIN = PICO_DEFAULT_LED_PIN;

// Hardware watchdog timeout — must be kicked every loop iteration.
// If the Pico firmware hangs, this triggers a full reset.
// Set well above the loop period (100ms) so normal operation never triggers it.
static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 1000;

int main() {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);  // solid on - program is running

    // Enable hardware watchdog — fires if main loop stops kicking it
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

    I2CSlave slave;
    slave.init();

    DriveMotorController motors;
    motors.init();

    SpaceResourcesMotor space_motor;
    space_motor.init();

    QuadratureEncoders encoders;
    encoders.init();

    MoistureSensor moisture;
    moisture.init();

    printf("Kibisis Pico Driver ready. I2C slave @ 0x%02X\n", SLAVE_ADDR);

    while (true) {
        // Kick hardware watchdog — proves firmware is alive
        watchdog_update();

        // Software comms timeout: if Pi stops sending commands, zero all motors
        // This is the primary safety mechanism for Pi crash / cable loss
        if (slave.hasCommandTimedOut()) {
            motors.setMotorA(0);
            motors.setMotorB(0);
            space_motor.setSpeed(0);
            printf("COMMS TIMEOUT — all motors stopped\n");
        } else {
            // Normal operation: apply whatever the Pi has commanded
            motors.setMotorA(slave.getMotorASpeed());
            motors.setMotorB(slave.getMotorBSpeed());
            space_motor.setSpeed(slave.getSpaceMotorSpeed());
        }

        // Read encoder counts from PIO state machines
        int32_t enc_a = encoders.getCountA();
        int32_t enc_b = encoders.getCountB();

        // On-demand moisture sample: Pi writes to MOISTURE_SAMPLE register to trigger
        if (slave.getMoistureSampleRequested()) {
            moisture.sample();
            slave.clearMoistureSample();
            printf("Moisture sampled: %u\n", moisture.getValue());
        }

        // Write latest values into shadow buffer and commit atomically
        slave.setEncoderA(enc_a);
        slave.setEncoderB(enc_b);
        slave.setMoistureValue(moisture.getValue());
        slave.setStatus(0x01);
        slave.commitRegisters();

        printf("Motor A: %d  Motor B: %d  Space: %d  Enc A: %ld  Enc B: %ld  Moisture: %u  Timeout: %s\n",
               slave.getMotorASpeed(), slave.getMotorBSpeed(), slave.getSpaceMotorSpeed(),
               enc_a, enc_b, moisture.getValue(),
               slave.hasCommandTimedOut() ? "YES" : "no");

        sleep_ms(100);
    }
}

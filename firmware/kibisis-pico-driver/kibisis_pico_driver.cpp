#include <stdio.h>
#include "pico/stdlib.h"
#include "i2c_slave.hpp"
#include "drive_motors.hpp"
#include "quadrature_encoders.hpp"
#include "space_resources_motor.hpp"
#include "ldr_sensors.hpp"
#include "status-light.hpp"

static constexpr uint LED_PIN = PICO_DEFAULT_LED_PIN;

int main()
{
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    I2CSlave slave;
    slave.init();
    printf("I2C Slave Enabled.\n");

    DriveMotors motors;
    motors.init();
    printf("Motors initialised.\n");

    LdrSensors ldrs;
    ldrs.init();
    printf("LDR sensors initialised.\n");

    StatusLight status_light;
    status_light.init();

    // SpaceResourcesMotor space_motor;
    // space_motor.init();

    printf("Kibisis firmware ready. I2C @ 0x%02X\n", SLAVE_ADDR);

    while (true)
    {
        // Drive motors
        int8_t a = slave.getMotorASpeed();
        int8_t b = slave.getMotorBSpeed();
        motors.setMotorA(a);
        motors.setMotorB(b);

        // LDR sample on demand
        if (slave.getLdrSampleTrigger()) {
            auto r = ldrs.sample();
            slave.setLdrAmbientA(r.ldr_a_ambient);
            slave.setLdrAmbientB(r.ldr_b_ambient);
            slave.setLdrIlluminatedA(r.ldr_a_illuminated);
            slave.setLdrIlluminatedB(r.ldr_b_illuminated);
            printf("LDR A: ambient=%u illuminated=%u  LDR B: ambient=%u illuminated=%u\n",
                   r.ldr_a_ambient, r.ldr_a_illuminated,
                   r.ldr_b_ambient, r.ldr_b_illuminated);
        }

        // space_motor.setSpeed(slave.getSpaceMotorSpeed());

        slave.setStatus(0x01);
        slave.commitRegisters();

        printf("A: %d  B: %d\n", a, b);
        sleep_ms(50);
    }
}
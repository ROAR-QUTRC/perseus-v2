#include <cstdio>

#include "drive_motors.hpp"
#include "i2c_slave.hpp"
#include "ldr_sensors.hpp"
#include "quadrature_encoders.hpp"
#include "space_resources_motor.hpp"
#include "status_light.hpp"

#include "pico/stdlib.h"

namespace
{
constexpr uint kLedPin = PICO_DEFAULT_LED_PIN;
}

int main()
{
    stdio_init_all();

    gpio_init(kLedPin);
    gpio_set_dir(kLedPin, GPIO_OUT);
    gpio_put(kLedPin, 1);

    I2CSlave slave;
    slave.init();
    printf("I2C slave enabled.\n");

    DriveMotors motors;
    motors.init();
    printf("Motors initialised.\n");

    LdrSensors ldrs;
    ldrs.init();
    printf("LDR sensors initialised.\n");

    StatusLight status_light;
    status_light.init();
    printf("Status light initialised.\n");

    // SpaceResourcesMotor space_motor;
    // space_motor.init();

    printf("Kibisis firmware ready. I2C @ 0x%02X\n", kibisis::kSlaveAddr);

    while (true)
    {
        const int8_t motor_a = slave.getMotorASpeed();
        const int8_t motor_b = slave.getMotorBSpeed();
        motors.setMotorA(motor_a);
        motors.setMotorB(motor_b);

        if (slave.getLdrSampleTrigger())
        {
            const LdrSensors::Reading reading = ldrs.sample();
            slave.setLdrAmbientA(reading.ldrAAmbient);
            slave.setLdrAmbientB(reading.ldrBAmbient);
            slave.setLdrIlluminatedA(reading.ldrAIlluminated);
            slave.setLdrIlluminatedB(reading.ldrBIlluminated);
            printf("LDR A: ambient=%u illuminated=%u  LDR B: ambient=%u illuminated=%u\n",
                   reading.ldrAAmbient, reading.ldrAIlluminated,
                   reading.ldrBAmbient, reading.ldrBIlluminated);
        }

        // space_motor.setSpeed(slave.getSpaceMotorSpeed());

        slave.setStatus(0x01);
        slave.commitRegisters();

        printf("Motor A: %d  Motor B: %d\n", motor_a, motor_b);
        sleep_ms(50);
    }
}
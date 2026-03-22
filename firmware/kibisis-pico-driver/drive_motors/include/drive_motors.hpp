#pragma once
#include "pico/stdlib.h"
#include <cstdint>

// DFRobot DRI0041
// Motor A: PWM=GP6  IN1=GP7  IN2=GP8
// Motor B: PWM=GP9  IN1=GP10 IN2=GP11

class DriveMotors
{
public:
    void init();
    void setMotorA(int8_t speed);
    void setMotorB(int8_t speed);

private:
    static constexpr uint PIN_A_PWM = 6;
    static constexpr uint PIN_A_IN1 = 7;
    static constexpr uint PIN_A_IN2 = 8;
    static constexpr uint PIN_B_PWM = 9;
    static constexpr uint PIN_B_IN1 = 10;
    static constexpr uint PIN_B_IN2 = 11;
    static constexpr uint PWM_WRAP  = 10000;

    void init_motor(uint pin_pwm, uint pin_in1, uint pin_in2);
    void set_motor(uint pin_pwm, uint pin_in1, uint pin_in2, int8_t speed);
};

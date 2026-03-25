#pragma once

#include <cstdint>

#include "pico/stdlib.h"

/// Dual brushed DC motor driver for Kibisis.
///
/// Hardware: DFRobot DRI0041 dual motor driver carrier.
///   Motor A: PWM=GP6  IN1=GP7  IN2=GP8
///   Motor B: PWM=GP9  IN1=GP10 IN2=GP11
///
/// Speed is expressed as int8_t in the range [-100, 100].
/// Positive = forward, negative = reverse, 0 = coast/brake.
class DriveMotors
{
public:
    void init();
    void setMotorA(int8_t speed);
    void setMotorB(int8_t speed);

private:
    static constexpr uint kPinAPwm = 6;
    static constexpr uint kPinAIn1 = 7;
    static constexpr uint kPinAIn2 = 8;
    static constexpr uint kPinBPwm = 9;
    static constexpr uint kPinBIn1 = 10;
    static constexpr uint kPinBIn2 = 11;

    /// PWM counter wrap value — gives 10 000 discrete duty-cycle steps.
    static constexpr uint kPwmWrap = 10'000;

    void initMotor(uint pinPwm, uint pinIn1, uint pinIn2);
    void setMotor(uint pinPwm, uint pinIn1, uint pinIn2, int8_t speed);
};
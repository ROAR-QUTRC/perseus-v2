#pragma once
#include "pico/stdlib.h"
#include <cstdint>

// GPIO pin for the DFRobot DFR0430 micro metal gear motor signal
// GP12 is free and not used by I2C (GP4/5) or drive motors (GP6-11)
static constexpr uint SPACE_MOTOR_PIN = 12;

// PPM signal parameters for DFR0430
// We use an explicit 3000us period to comfortably fit the 2500us max pulse.
// Frequency is derived from the period rather than set directly.
static constexpr uint16_t SPACE_MOTOR_PERIOD_US  = 3000;  // us - gives ~333Hz
static constexpr uint16_t SPACE_MOTOR_PULSE_MIN  = 500;   // us - full clockwise
static constexpr uint16_t SPACE_MOTOR_PULSE_MID  = 1500;  // us - stop
static constexpr uint16_t SPACE_MOTOR_PULSE_MAX  = 2500;  // us - full anticlockwise

class SpaceResourcesMotor {
public:
    SpaceResourcesMotor();

    void init();

    // Set speed: -100 (full CW) to 0 (stop) to +100 (full CCW)
    void setSpeed(int8_t speed);

    // Explicitly stop the motor (sends mid pulse)
    void stop();

    int8_t getSpeed() const { return current_speed_; }

private:
    void setPulseUs(uint16_t pulse_us);

    uint pwm_slice_;
    uint pwm_channel_;
    uint32_t pwm_wrap_;

    int8_t current_speed_ = 0;
};
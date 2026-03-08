#pragma once
#include "pico/stdlib.h"
#include <cstdint>

// Pin assignments for DFRobot DRI0041 2x7A motor driver
// Grouped on consecutive GPIO pins for clean wiring
namespace DriveMotorPins {
    // Motor A
    static constexpr uint IN1 = 6;
    static constexpr uint IN2 = 7;
    static constexpr uint ENA = 8;   // PWM capable

    // Motor B
    static constexpr uint IN3 = 9;
    static constexpr uint IN4 = 10;
    static constexpr uint ENB = 11;  // PWM capable
}

// PWM configuration
static constexpr uint     DRIVE_PWM_FREQ_HZ  = 20000;  // 20kHz - above audible range
static constexpr uint16_t DRIVE_PWM_MAX      = 10000;  // 100% duty cycle

// Brake hold time when changing direction (ms)
// DRI0041 datasheet recommends >100ms before switching direction
static constexpr uint32_t DRIVE_BRAKE_MS = 100;

// Represents a single drive motor channel on the DRI0041
class DriveMotor {
public:
    DriveMotor(uint pin_in1, uint pin_in2, uint pin_en);

    void init();

    // Set speed as a percentage: -100 (full reverse) to +100 (full forward)
    // Automatically brakes before reversing direction
    void setSpeed(int8_t speed);

    // Immediately stop and hold (brake)
    void brake();

    int8_t getSpeed() const { return current_speed_; }

private:
    void setPwmDuty(uint16_t duty);
    void setForward();
    void setReverse();

    uint pin_in1_;
    uint pin_in2_;
    uint pin_en_;

    uint pwm_slice_;
    uint pwm_channel_;

    int8_t current_speed_ = 0;
};

// Manages both drive motor channels together
class DriveMotorController {
public:
    DriveMotorController();

    void init();

    // Set speeds: -100 to +100
    void setMotorA(int8_t speed);
    void setMotorB(int8_t speed);
    void brakeAll();

private:
    DriveMotor motor_a_;
    DriveMotor motor_b_;
};

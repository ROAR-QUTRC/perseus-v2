// space_resources_motor.hpp
// DFR0399/DFR4030 rack-and-pinion motor driver for Kibisis Pico firmware.
// Accepts a signed speed value: positive = extend, negative = retract, 0 = stop.
// Outputs a servo-style PWM signal (50 Hz, 500–2500 µs) on a single GPIO pin.

#pragma once

#include <cstdint>

#include "hardware/pwm.h"
#include "pico/stdlib.h"

/// @brief Driver for the DFR0399/DFR4030 continuous-rotation motor module.
///
/// Matches the init()/setSpeed() interface used by other Kibisis drivers.
/// Speed is a signed 8-bit value:
///   +1 to +100  - extend  (maps dead-band edge to full CW speed)
///   -1 to -100  - retract (maps dead-band edge to full CCW speed)
///    0          - stop    (dead-band centre, 1500 µs)
class SRMotor
{
public:
    /// @brief Initialise PWM hardware. Must be called before setSpeed().
    void init();

    /// @brief Set motor speed and direction.
    /// @param speed -100 to +100. Positive = extend, negative = retract, 0 = stop.
    void setSpeed(int8_t speed);

private:
    static constexpr uint32_t kSysClkHz       = 125'000'000U;
    static constexpr float    kClkDiv         = 64.0F;
    static constexpr uint32_t kWrapVal        = 2441U;

    static constexpr uint32_t kPwStopUs       = 1500U;
    static constexpr uint32_t kPwExtendMaxUs  = 2500U;
    static constexpr uint32_t kPwRetractMaxUs = 500U;
    static constexpr uint32_t kPwDeadHighUs   = 1600U;
    static constexpr uint32_t kPwDeadLowUs    = 1400U;

    static constexpr uint32_t kSignalPin      = 16U;

    static uint32_t us_to_level(const uint32_t &pulse_us);
    void set_pulse_us(const uint32_t &pulse_us) const;
};
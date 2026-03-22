#pragma once

#include <algorithm>
#include <cstdint>

#include "hardware/pwm.h"
#include "pico/stdlib.h"

/// DFRobot DFR0430 brushless ESC driver via PPM signal.
///
/// PPM pulse range: 500 µs – 2500 µs within a 3000 µs period.
///   Neutral (stop): 1500 µs
///   Full forward:   2500 µs
///   Full reverse:    500 µs
///
/// Speed is expressed as int8_t in the range [-100, 100].
/// 0 = neutral (1500 µs pulse).
class SpaceResourcesMotor
{
public:
    explicit SpaceResourcesMotor(uint pin = 15) : pin_(pin) {}

    void init();

    /// Set motor speed in the range [-100, 100]. 0 = neutral/stop.
    void setSpeed(int8_t speed);

    void stop() { setSpeed(0); }

private:
    uint pin_;
    uint slice_ = 0;
    uint chan_  = 0;

    static constexpr uint32_t kPeriodUs  = 3000;
    static constexpr uint32_t kNeutralUs = 1500;
    static constexpr uint32_t kMinUs     = 500;
    static constexpr uint32_t kMaxUs     = 2500;

    void setPulseUs(uint32_t pulseUs);
};
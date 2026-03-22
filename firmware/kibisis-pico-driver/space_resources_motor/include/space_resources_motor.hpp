#pragma once
#include <algorithm>
#include <cstdint>

#include "hardware/pwm.h"
#include "pico/stdlib.h"

// DFRobot DFR0430 brushless ESC controlled via PPM signal.
//
// PPM pulse range: 500us - 2500us within a 3000us period.
// Neutral (stop): 1500us
// Full forward:   2500us
// Full reverse:   500us
//
// Speed is expressed as int8_t: -100 to +100
// 0 = neutral (1500us pulse)

class SpaceResourcesMotor
{
public:
    // GP12 by default — change if your wiring differs
    explicit SpaceResourcesMotor(uint pin = 15)
        : pin_(pin)
    {
    }

    void init();
    void setSpeed(int8_t speed);  // -100 to +100
    void stop() { setSpeed(0); }

private:
    uint pin_;
    uint slice_;
    uint chan_;

    static constexpr uint32_t PERIOD_US = 3000;
    static constexpr uint32_t NEUTRAL_US = 1500;
    static constexpr uint32_t MIN_US = 500;
    static constexpr uint32_t MAX_US = 2500;
    static constexpr uint32_t MOTOR_SYS_CLK_HZ = 150'000'000;

    void setPulseUs(uint32_t pulse_us);
};
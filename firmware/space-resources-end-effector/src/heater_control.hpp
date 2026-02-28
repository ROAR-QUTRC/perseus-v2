#pragma once

#include <cstdint>

#include "pico/types.h"

namespace heater_control {

/// GPIO PWM heater control at 1 kHz
class HeaterControl {
public:
    explicit HeaterControl(uint8_t gpio_pin = 15);

    /// Initialize PWM on the GPIO pin
    void init();

    /// Set duty cycle (0-255)
    void set_duty(uint8_t duty);

    /// Turn off heater
    void off();

    uint8_t current_duty() const { return _duty; }

private:
    uint8_t _gpio_pin;
    uint _slice;
    uint _channel;
    uint8_t _duty;
};

}  // namespace heater_control

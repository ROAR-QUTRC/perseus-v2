#include "heater_control.hpp"

#include <cstdio>

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

namespace heater_control
{

    HeaterControl::HeaterControl(uint8_t gpio_pin)
        : _gpio_pin(gpio_pin),
          _slice(0),
          _channel(0),
          _duty(0)
    {
    }

    void HeaterControl::init()
    {
        gpio_set_function(_gpio_pin, GPIO_FUNC_PWM);
        _slice = pwm_gpio_to_slice_num(_gpio_pin);
        _channel = pwm_gpio_to_channel(_gpio_pin);

        // Configure for ~1 kHz: 125MHz / (125 * 1000) = 1 kHz
        // wrap = 999, divider = 125.0
        pwm_set_wrap(_slice, 999);
        pwm_set_clkdiv(_slice, 125.0f);
        pwm_set_chan_level(_slice, _channel, 0);
        pwm_set_enabled(_slice, true);

        printf("Heater control initialized on GPIO %u (slice %u, channel %u)\n",
               _gpio_pin, _slice, _channel);
    }

    void HeaterControl::set_duty(uint8_t duty)
    {
        _duty = duty;
        // Map 0-255 to 0-999 PWM counts
        uint16_t level = (static_cast<uint16_t>(duty) * 999) / 255;
        pwm_set_chan_level(_slice, _channel, level);
    }

    void HeaterControl::off()
    {
        set_duty(0);
    }

}  // namespace heater_control

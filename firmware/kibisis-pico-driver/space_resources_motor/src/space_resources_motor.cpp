// space_resources_motor.cpp
// Implementation of the SRMotor rack-and-pinion motor driver.

#include "space_resources_motor.hpp"

#include <cstdint>

#include "hardware/pwm.h"
#include "pico/stdlib.h"

void SRMotor::init()
{
    gpio_set_function(kSignalPin, GPIO_FUNC_PWM);
    const uint32_t slice = pwm_gpio_to_slice_num(kSignalPin);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, kClkDiv);
    pwm_config_set_wrap(&cfg, kWrapVal);
    pwm_init(slice, &cfg, true);

    set_pulse_us(kPwStopUs);
}

void SRMotor::setSpeed(const int8_t speed)
{
    if (speed == 0)
    {
        set_pulse_us(kPwStopUs);
        return;
    }

    const int8_t clamped = (speed > 100) ? 100 : (speed < -100) ? -100 : speed;

    if (clamped > 0)
    {
        const uint32_t magnitude = static_cast<uint32_t>(clamped);
        const uint32_t pw = kPwDeadHighUs
                            + (magnitude * (kPwExtendMaxUs - kPwDeadHighUs)) / 100U;
        set_pulse_us(pw);
    }
    else
    {
        const uint32_t magnitude = static_cast<uint32_t>(-clamped);
        const uint32_t pw = kPwDeadLowUs
                            - (magnitude * (kPwDeadLowUs - kPwRetractMaxUs)) / 100U;
        set_pulse_us(pw);
    }
}

uint32_t SRMotor::us_to_level(const uint32_t &pulse_us)
{
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(pulse_us) * kSysClkHz)
        / (static_cast<uint64_t>(kClkDiv) * 1'000'000ULL));
}

void SRMotor::set_pulse_us(const uint32_t &pulse_us) const
{
    const uint32_t slice   = pwm_gpio_to_slice_num(kSignalPin);
    const uint32_t channel = pwm_gpio_to_channel(kSignalPin);
    pwm_set_chan_level(slice, channel, us_to_level(pulse_us));
}
#include "space_resources_motor.hpp"

void SpaceResourcesMotor::init()
{
    gpio_set_function(pin_, GPIO_FUNC_PWM);
    slice_ = pwm_gpio_to_slice_num(pin_);
    chan_ = pwm_gpio_to_channel(pin_);

    // We want a 3000us period.
    // PWM wrap = (SYS_CLK_HZ / clkdiv) * period_s - 1
    // Use clkdiv=150 -> 1MHz tick (1us resolution), wrap=2999
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 150.0f);       // 150MHz / 150 = 1MHz = 1us/tick
    pwm_config_set_wrap(&cfg, PERIOD_US - 1);  // 2999
    pwm_init(slice_, &cfg, true);

    setPulseUs(NEUTRAL_US);
}

void SpaceResourcesMotor::setSpeed(int8_t speed)
{
    int8_t clamped = std::clamp(speed, (int8_t)-100, (int8_t)100);
    uint32_t pulse_us;
    if (clamped >= 0)
    {
        // 0..+100 -> 1500..2500us
        pulse_us = NEUTRAL_US + (uint32_t)(clamped * (int32_t)(MAX_US - NEUTRAL_US) / 100);
    }
    else
    {
        // -100..0 -> 500..1500us
        pulse_us = NEUTRAL_US + (int32_t)(clamped * (int32_t)(NEUTRAL_US - MIN_US) / 100);
    }
    setPulseUs(pulse_us);
}

void SpaceResourcesMotor::setPulseUs(uint32_t pulse_us)
{
    pwm_set_chan_level(slice_, chan_, pulse_us);
}
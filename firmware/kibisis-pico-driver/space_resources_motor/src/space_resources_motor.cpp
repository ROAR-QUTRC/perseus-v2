#include "space_resources_motor.hpp"

void SpaceResourcesMotor::init()
{
    gpio_set_function(pin_, GPIO_FUNC_PWM);
    slice_ = pwm_gpio_to_slice_num(pin_);
    chan_ = pwm_gpio_to_channel(pin_);

    // 1 µs resolution: 150 MHz / 150 = 1 MHz tick, wrap at 2999 → 3000 µs period.
    static constexpr float kClkDiv = 150.0f;
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, kClkDiv);
    pwm_config_set_wrap(&cfg, kPeriodUs - 1);
    pwm_init(slice_, &cfg, true);

    setPulseUs(kNeutralUs);
}

void SpaceResourcesMotor::setSpeed(const int8_t speed)
{
    const int8_t clamped = std::clamp(speed, static_cast<int8_t>(-100), static_cast<int8_t>(100));
    uint32_t pulseUs = 0;

    if (clamped >= 0)
    {
        // 0..+100 → 1500..2500 µs
        pulseUs = kNeutralUs
                  + static_cast<uint32_t>(clamped)
                    * (kMaxUs - kNeutralUs) / 100;
    }
    else
    {
        // -100..0 → 500..1500 µs
        pulseUs = static_cast<uint32_t>(
            static_cast<int32_t>(kNeutralUs)
            + static_cast<int32_t>(clamped)
              * static_cast<int32_t>(kNeutralUs - kMinUs) / 100);
    }

    setPulseUs(pulseUs);
}

void SpaceResourcesMotor::setPulseUs(const uint32_t pulseUs)
{
    pwm_set_chan_level(slice_, chan_, pulseUs);
}
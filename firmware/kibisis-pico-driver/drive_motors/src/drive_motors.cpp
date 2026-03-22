#include "drive_motors.hpp"


#include <cstdlib>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

void DriveMotors::init()
{
    initMotor(kPinAPwm, kPinAIn1, kPinAIn2);
    initMotor(kPinBPwm, kPinBIn1, kPinBIn2);
}

void DriveMotors::initMotor(const uint pinPwm, const uint pinIn1, const uint pinIn2)
{
    gpio_init(pinIn1);
    gpio_set_dir(pinIn1, GPIO_OUT);
    gpio_put(pinIn1, 0);

    gpio_init(pinIn2);
    gpio_set_dir(pinIn2, GPIO_OUT);
    gpio_put(pinIn2, 0);

    gpio_set_function(pinPwm, GPIO_FUNC_PWM);

    const uint    slice  = pwm_gpio_to_slice_num(pinPwm);
    pwm_config    cfg    = pwm_get_default_config();
    const float   clkdiv = static_cast<float>(clock_get_hz(clk_sys))
                           / (static_cast<float>(kPwmWrap) * 20000.0f);
    pwm_config_set_clkdiv(&cfg, clkdiv);
    pwm_config_set_wrap(&cfg, kPwmWrap);
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(pinPwm, 0);
}

void DriveMotors::setMotor(const uint pinPwm, const uint pinIn1, const uint pinIn2,
                           const int8_t speed)
{
    const uint32_t duty = static_cast<uint32_t>(std::abs(speed)) * kPwmWrap / 100;

    if (speed > 0)
    {
        gpio_put(pinIn1, 1);
        gpio_put(pinIn2, 0);
    }
    else if (speed < 0)
    {
        gpio_put(pinIn1, 0);
        gpio_put(pinIn2, 1);
    }
    else
    {
        gpio_put(pinIn1, 0);
        gpio_put(pinIn2, 0);
    }

    pwm_set_gpio_level(pinPwm, duty);
}

void DriveMotors::setMotorA(const int8_t speed) { setMotor(kPinAPwm, kPinAIn1, kPinAIn2, speed); }
void DriveMotors::setMotorB(const int8_t speed) { setMotor(kPinBPwm, kPinBIn1, kPinBIn2, speed); }
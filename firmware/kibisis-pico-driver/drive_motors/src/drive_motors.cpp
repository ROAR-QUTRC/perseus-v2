#include "drive_motors.hpp"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include <cstdlib>

void DriveMotors::init()
{
    init_motor(PIN_A_PWM, PIN_A_IN1, PIN_A_IN2);
    init_motor(PIN_B_PWM, PIN_B_IN1, PIN_B_IN2);
}

void DriveMotors::init_motor(uint pin_pwm, uint pin_in1, uint pin_in2)
{
    gpio_init(pin_in1); gpio_set_dir(pin_in1, GPIO_OUT); gpio_put(pin_in1, 0);
    gpio_init(pin_in2); gpio_set_dir(pin_in2, GPIO_OUT); gpio_put(pin_in2, 0);
    gpio_set_function(pin_pwm, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin_pwm);
    pwm_config cfg = pwm_get_default_config();
    float clkdiv = static_cast<float>(clock_get_hz(clk_sys)) / (PWM_WRAP * 20000.0f);
    pwm_config_set_clkdiv(&cfg, clkdiv);
    pwm_config_set_wrap(&cfg, PWM_WRAP);
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(pin_pwm, 0);
}

void DriveMotors::set_motor(uint pin_pwm, uint pin_in1, uint pin_in2, int8_t speed)
{
    uint32_t duty = static_cast<uint32_t>(std::abs(speed)) * PWM_WRAP / 100;
    if (speed > 0)      { gpio_put(pin_in1, 1); gpio_put(pin_in2, 0); }
    else if (speed < 0) { gpio_put(pin_in1, 0); gpio_put(pin_in2, 1); }
    else                { gpio_put(pin_in1, 0); gpio_put(pin_in2, 0); }
    pwm_set_gpio_level(pin_pwm, duty);
}

void DriveMotors::setMotorA(int8_t speed) { set_motor(PIN_A_PWM, PIN_A_IN1, PIN_A_IN2, speed); }
void DriveMotors::setMotorB(int8_t speed) { set_motor(PIN_B_PWM, PIN_B_IN1, PIN_B_IN2, speed); }

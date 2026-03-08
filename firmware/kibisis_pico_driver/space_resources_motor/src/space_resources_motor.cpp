#include "space_resources_motor.hpp"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

SpaceResourcesMotor::SpaceResourcesMotor()
    : pwm_slice_(pwm_gpio_to_slice_num(SPACE_MOTOR_PIN))
    , pwm_channel_(pwm_gpio_to_channel(SPACE_MOTOR_PIN))
    , pwm_wrap_(0)
{}

void SpaceResourcesMotor::init() {
    gpio_set_function(SPACE_MOTOR_PIN, GPIO_FUNC_PWM);

    // Strategy: set each PWM count = 1us by dividing sys_clk down to 1MHz.
    // clk_div = sys_clk_hz / 1_000_000
    // wrap    = period_us - 1
    // This gives direct 1us resolution so pulse_us maps directly to PWM level.
    //
    // At 150MHz: clk_div = 150, wrap = 2999 for 3000us period
    // RP2350 max clk_div is 255.9 so 150 is well within range.
    uint32_t sys_hz  = clock_get_hz(clk_sys);
    float    clk_div = (float)sys_hz / 1000000.0f;

    pwm_wrap_ = SPACE_MOTOR_PERIOD_US - 1;

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, clk_div);
    pwm_config_set_wrap(&cfg, pwm_wrap_);
    pwm_init(pwm_slice_, &cfg, true);

    stop();
}

void SpaceResourcesMotor::setSpeed(int8_t speed) {
    if (speed > 100)  speed = 100;
    if (speed < -100) speed = -100;

    current_speed_ = speed;

    uint16_t pulse_us;
    if (speed == 0) {
        pulse_us = SPACE_MOTOR_PULSE_MID;
    } else if (speed > 0) {
        // Positive speed: mid (1500us) to max (2500us) = anticlockwise
        pulse_us = SPACE_MOTOR_PULSE_MID +
                   (uint16_t)((speed * (SPACE_MOTOR_PULSE_MAX - SPACE_MOTOR_PULSE_MID)) / 100);
    } else {
        // Negative speed: mid (1500us) to min (500us) = clockwise
        pulse_us = SPACE_MOTOR_PULSE_MID -
                   (uint16_t)((-speed * (SPACE_MOTOR_PULSE_MID - SPACE_MOTOR_PULSE_MIN)) / 100);
    }

    setPulseUs(pulse_us);
}

void SpaceResourcesMotor::stop() {
    current_speed_ = 0;
    setPulseUs(SPACE_MOTOR_PULSE_MID);
}

void SpaceResourcesMotor::setPulseUs(uint16_t pulse_us) {
    // With 1us resolution, duty count = pulse_us directly
    pwm_set_chan_level(pwm_slice_, pwm_channel_, pulse_us);
}
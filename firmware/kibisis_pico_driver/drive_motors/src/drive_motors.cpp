#include "drive_motors.hpp"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include <cstdlib>  // abs()

// ---------------------------------------------------------------------------
// DriveMotor
// ---------------------------------------------------------------------------

DriveMotor::DriveMotor(uint pin_in1, uint pin_in2, uint pin_en)
    : pin_in1_(pin_in1)
    , pin_in2_(pin_in2)
    , pin_en_(pin_en)
    , pwm_slice_(pwm_gpio_to_slice_num(pin_en))
    , pwm_channel_(pwm_gpio_to_channel(pin_en))
{}

void DriveMotor::init() {
    // Set output value BEFORE setting direction to prevent floating
    // during the brief window between gpio_init and gpio_set_dir
    gpio_init(pin_in1_);
    gpio_put(pin_in1_, 0);
    gpio_set_dir(pin_in1_, GPIO_OUT);

    gpio_init(pin_in2_);
    gpio_put(pin_in2_, 0);
    gpio_set_dir(pin_in2_, GPIO_OUT);

    // EN pin - configure as PWM
    gpio_set_function(pin_en_, GPIO_FUNC_PWM);

    // Configure PWM: 20kHz, wrap at DRIVE_PWM_MAX
    // sys_clock / (divider * wrap) = freq
    // 150MHz / (divider * 10000) = 20000Hz  =>  divider = 0.75
    pwm_config cfg = pwm_get_default_config();
    float divider = (float)clock_get_hz(clk_sys) / (DRIVE_PWM_FREQ_HZ * DRIVE_PWM_MAX);
    pwm_config_set_clkdiv(&cfg, divider);
    pwm_config_set_wrap(&cfg, DRIVE_PWM_MAX - 1);
    pwm_init(pwm_slice_, &cfg, true);

    setPwmDuty(0);
    brake();
}

void DriveMotor::setSpeed(int8_t speed) {
    if (speed > 100)  speed = 100;
    if (speed < -100) speed = -100;

    // Brake before reversing direction per DRI0041 spec
    if ((current_speed_ > 0 && speed < 0) ||
        (current_speed_ < 0 && speed > 0)) {
        brake();
        sleep_ms(DRIVE_BRAKE_MS);
    }

    current_speed_ = speed;

    if (speed == 0) {
        brake();
        return;
    }

    uint16_t duty = (uint16_t)((abs(speed) * DRIVE_PWM_MAX) / 100);

    if (speed > 0) {
        setForward();
    } else {
        setReverse();
    }

    setPwmDuty(duty);
}

void DriveMotor::brake() {
    // IN1=0, IN2=0 = brake per DRI0041 truth table
    gpio_put(pin_in1_, 0);
    gpio_put(pin_in2_, 0);
    setPwmDuty(0);
    current_speed_ = 0;
}

void DriveMotor::setForward() {
    // IN1=1, IN2=0 = forward per DRI0041 truth table
    gpio_put(pin_in1_, 1);
    gpio_put(pin_in2_, 0);
}

void DriveMotor::setReverse() {
    // IN1=0, IN2=1 = reverse per DRI0041 truth table
    gpio_put(pin_in1_, 0);
    gpio_put(pin_in2_, 1);
}

void DriveMotor::setPwmDuty(uint16_t duty) {
    pwm_set_chan_level(pwm_slice_, pwm_channel_, duty);
}

// ---------------------------------------------------------------------------
// DriveMotorController
// ---------------------------------------------------------------------------

DriveMotorController::DriveMotorController()
    : motor_a_(DriveMotorPins::IN1, DriveMotorPins::IN2, DriveMotorPins::ENA)
    , motor_b_(DriveMotorPins::IN3, DriveMotorPins::IN4, DriveMotorPins::ENB)
{}

void DriveMotorController::init() {
    motor_a_.init();
    motor_b_.init();
}

void DriveMotorController::setMotorA(int8_t speed) {
    motor_a_.setSpeed(speed);
}

void DriveMotorController::setMotorB(int8_t speed) {
    motor_b_.setSpeed(speed);
}

void DriveMotorController::brakeAll() {
    motor_a_.brake();
    motor_b_.brake();
}
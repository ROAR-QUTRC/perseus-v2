#include "servo_driver.hpp"

#include <cstdio>

#include "pico/stdlib.h"

namespace servo_driver {

// PCA9685 registers
constexpr uint8_t REG_MODE1 = 0x00;
constexpr uint8_t REG_PRESCALE = 0xFE;
constexpr uint8_t REG_LED0_ON_L = 0x06;
constexpr uint8_t REG_ALL_LED_OFF_H = 0xFD;

// PCA9685 mode bits
constexpr uint8_t MODE1_SLEEP = 0x10;
constexpr uint8_t MODE1_AI = 0x20;    // Auto-increment
constexpr uint8_t MODE1_RESTART = 0x80;

// Servo pulse widths in microseconds at 50 Hz (20ms period)
constexpr float SERVO_MIN_US = 1000.0f;   // Full reverse
constexpr float SERVO_MID_US = 1500.0f;   // Stopped
constexpr float SERVO_MAX_US = 2000.0f;   // Full forward

// PCA9685 has 4096 counts per period
constexpr float COUNTS_PER_US = 4096.0f / 20000.0f;  // At 50 Hz

ServoDriver::ServoDriver(i2c_inst_t* i2c, uint8_t address)
    : _i2c(i2c), _address(address), _ok(false)
{
}

bool ServoDriver::init()
{
    // Put to sleep for prescale change
    write_reg(REG_MODE1, MODE1_SLEEP);
    sleep_ms(5);

    // Set prescale for 50 Hz: prescale = round(25MHz / (4096 * 50)) - 1 = 121
    write_reg(REG_PRESCALE, 121);
    sleep_ms(5);

    // Wake up with auto-increment enabled
    write_reg(REG_MODE1, MODE1_AI);
    sleep_ms(5);

    // Restart
    write_reg(REG_MODE1, MODE1_AI | MODE1_RESTART);
    sleep_ms(5);

    // Verify mode register
    uint8_t mode = read_reg(REG_MODE1);
    _ok = (mode & MODE1_AI) != 0;

    if (_ok) {
        printf("PCA9685 servo driver initialized at 0x%02X\n", _address);
        stop_all();
    } else {
        printf("PCA9685 init failed at 0x%02X\n", _address);
    }

    return _ok;
}

void ServoDriver::set_speed(uint8_t channel, float speed)
{
    if (channel > 15 || !_ok) {
        return;
    }

    // Clamp speed to -1.0..+1.0
    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;

    // Map speed to pulse width: -1.0 -> 1000us, 0.0 -> 1500us, +1.0 -> 2000us
    float pulse_us = SERVO_MID_US + speed * (SERVO_MAX_US - SERVO_MID_US);

    // Convert to PCA9685 counts
    uint16_t off_count = static_cast<uint16_t>(pulse_us * COUNTS_PER_US);
    set_pwm(channel, 0, off_count);
}

void ServoDriver::stop_all()
{
    for (uint8_t ch = 0; ch < 16; ch++) {
        set_speed(ch, 0.0f);
    }
}

void ServoDriver::set_pwm(uint8_t channel, uint16_t on, uint16_t off)
{
    uint8_t reg = REG_LED0_ON_L + 4 * channel;
    uint8_t buf[5] = {
        reg,
        static_cast<uint8_t>(on & 0xFF),
        static_cast<uint8_t>((on >> 8) & 0x0F),
        static_cast<uint8_t>(off & 0xFF),
        static_cast<uint8_t>((off >> 8) & 0x0F),
    };
    i2c_write_blocking(_i2c, _address, buf, 5, false);
}

bool ServoDriver::write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    int ret = i2c_write_blocking(_i2c, _address, buf, 2, false);
    return ret >= 0;
}

uint8_t ServoDriver::read_reg(uint8_t reg)
{
    uint8_t value = 0;
    i2c_write_blocking(_i2c, _address, &reg, 1, true);
    i2c_read_blocking(_i2c, _address, &value, 1, false);
    return value;
}

}  // namespace servo_driver

#pragma once

#include <cstdint>

#include "hardware/i2c.h"

namespace servo_driver
{

    /// PCA9685 servo driver at 50 Hz PWM for continuous rotation servos
    class ServoDriver
    {
    public:
        ServoDriver(i2c_inst_t* i2c, uint8_t address = 0x44);

        /// Initialize PCA9685 with 50 Hz prescale
        bool init();

        /// Set servo speed on channel. speed: -1.0 to +1.0 (0.0 = stopped)
        void set_speed(uint8_t channel, float speed);

        /// Stop all channels
        void stop_all();

        bool is_ok() const { return _ok; }

    private:
        i2c_inst_t* _i2c;
        uint8_t _address;
        bool _ok;

        void set_pwm(uint8_t channel, uint16_t on, uint16_t off);
        bool write_reg(uint8_t reg, uint8_t value);
        uint8_t read_reg(uint8_t reg);
    };

}  // namespace servo_driver

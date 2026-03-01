#include "kibisis_pwm_converter.hpp"

#include <array>
#include <cstring>

#include "config.hpp"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/i2c_slave.h"
#include "pico/stdlib.h"

// ─────────────────────────────────────────────────────────────────────────────
// kibisis_pwm_converter.cpp
//
// All implementation details are file-scoped (anonymous namespace).
// Only the three functions declared in kibisis_pwm_converter.hpp are public.
// ─────────────────────────────────────────────────────────────────────────────

namespace
{

    // ── PWM pin table (indexed by function byte 0x00–0x03) ───────────────────────

    constexpr std::array<uint, PWM_CHANNEL_COUNT> PWM_PINS = {
        PWM_PIN_LEFT_FORWARD,
        PWM_PIN_LEFT_BACKWARD,
        PWM_PIN_RIGHT_FORWARD,
        PWM_PIN_RIGHT_BACKWARD,
    };

    // ── Shared state (written from ISR, consumed by main loop) ───────────────────

    volatile uint16_t pwm_target[PWM_CHANNEL_COUNT] = {};
    volatile bool pwm_dirty[PWM_CHANNEL_COUNT] = {};

    volatile uint16_t encoder_left = 0;
    volatile uint16_t encoder_right = 0;

    // ── I2C write state machine ───────────────────────────────────────────────────

    enum class WriteState : uint8_t
    {
        WaitFunction,  // expecting a function or encoder register byte
        WaitPwmHi,     // received function byte, waiting for PWM high byte
        WaitPwmLo,     // received PWM high byte, waiting for PWM low byte
    };

    struct I2cContext
    {
        // Write path
        WriteState write_state = WriteState::WaitFunction;
        uint8_t function = 0;  // latched function byte (0x00–0x03)
        uint8_t pwm_hi = 0;    // latched high byte while waiting for low

        // Read path
        uint8_t read_reg = 0;
        bool read_reg_valid = false;
        uint8_t read_byte_idx = 0;  // 0 = send high byte next, 1 = send low byte
    };

    I2cContext ctx;

    // ── I2C slave event handler (runs in IRQ – must be fast) ─────────────────────

    void i2c_slave_handler(i2c_inst_t* i2c, i2c_slave_event_t event)
    {
        switch (event)
        {
        case I2C_SLAVE_RECEIVE:
        {
            const uint8_t byte = i2c_read_byte_raw(i2c);

            switch (ctx.write_state)
            {
            case WriteState::WaitFunction:
                if (byte == REG_ENCODER_LEFT || byte == REG_ENCODER_RIGHT)
                {
                    // Write phase of an encoder read transaction
                    ctx.read_reg = byte;
                    ctx.read_reg_valid = true;
                }
                else if (byte < PWM_CHANNEL_COUNT)
                {
                    // Motor command – latch function and collect two PWM bytes
                    ctx.function = byte;
                    ctx.write_state = WriteState::WaitPwmHi;
                }
                // Unknown bytes silently ignored
                break;

            case WriteState::WaitPwmHi:
                ctx.pwm_hi = byte;
                ctx.write_state = WriteState::WaitPwmLo;
                break;

            case WriteState::WaitPwmLo:
            {
                const uint16_t value = (static_cast<uint16_t>(ctx.pwm_hi) << 8) | byte;
                pwm_target[ctx.function] = value;
                pwm_dirty[ctx.function] = true;
                ctx.write_state = WriteState::WaitFunction;
                break;
            }
            }
            break;
        }

        case I2C_SLAVE_REQUEST:
        {
            // Return the requested encoder value as big-endian uint16
            const uint16_t val = ctx.read_reg_valid
                                     ? (ctx.read_reg == REG_ENCODER_LEFT ? encoder_left : encoder_right)
                                     : 0;

            if (ctx.read_byte_idx == 0)
            {
                i2c_write_byte_raw(i2c, static_cast<uint8_t>(val >> 8));
                ctx.read_byte_idx = 1;
            }
            else
            {
                i2c_write_byte_raw(i2c, static_cast<uint8_t>(val & 0xFF));
                ctx.read_byte_idx = 0;
                ctx.read_reg_valid = false;
            }
            break;
        }

        case I2C_SLAVE_FINISH:
            // STOP/RESTART – reset state machine for next transaction
            ctx.write_state = WriteState::WaitFunction;
            ctx.read_byte_idx = 0;
            break;

        default:
            break;
        }
    }

    // ── PWM helpers ───────────────────────────────────────────────────────────────

    void pwm_channel_init(uint gpio)
    {
        gpio_set_function(gpio, GPIO_FUNC_PWM);

        const uint slice = pwm_gpio_to_slice_num(gpio);
        pwm_config config = pwm_get_default_config();
        pwm_config_set_clkdiv_int(&config, PWM_CLOCK_DIV);
        pwm_config_set_wrap(&config, PWM_WRAP);
        pwm_init(slice, &config, /*start=*/true);
        pwm_set_gpio_level(gpio, 0);
    }

    void pwm_channel_set(uint gpio, uint16_t level)
    {
        if (level > PWM_WRAP)
            level = PWM_WRAP;
        pwm_set_gpio_level(gpio, level);
    }

}  // anonymous namespace

// ── Public API (kibisis namespace) ────────────────────────────────────────────

namespace kibisis
{

    void pwm_converter_init()
    {
        for (const uint pin : PWM_PINS)
            pwm_channel_init(pin);

        gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(I2C_SDA_PIN);  // backup; prefer external 4.7 kΩ pull-ups
        gpio_pull_up(I2C_SCL_PIN);

        i2c_slave_init(I2C_INSTANCE, I2C_SLAVE_ADDR, &i2c_slave_handler);

        ctx = I2cContext{};
    }

    void pwm_converter_apply_updates()
    {
        for (uint8_t i = 0; i < PWM_CHANNEL_COUNT; ++i)
        {
            if (pwm_dirty[i])
            {
                const uint16_t level = pwm_target[i];  // 16-bit aligned read is atomic on CM33
                pwm_dirty[i] = false;
                pwm_channel_set(PWM_PINS[i], level);
            }
        }
    }

    void pwm_converter_set_encoder(uint16_t left_count, uint16_t right_count)
    {
        // 16-bit aligned writes are atomic on Cortex-M33 – no critical section needed
        encoder_left = left_count;
        encoder_right = right_count;
    }

}  // namespace kibisis

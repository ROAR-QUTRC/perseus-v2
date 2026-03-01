#pragma once

#include <cstdint>

// ─────────────────────────────────────────────────────────────────────────────
// kibisis_pwm_converter.hpp
//
// Receives motor PWM commands over I2C from the Pi 5 master and drives
// hardware PWM outputs on the Pico 2. Also serves encoder counts back to
// the master on read requests.
//
// Usage:
//   1. Call kibisis_pwm_converter_init() once at startup, after i2c_init().
//   2. Call kibisis_pwm_converter_apply_updates() every main loop iteration.
//   3. Call kibisis_pwm_converter_set_encoder() from your encoder module
//      whenever fresh counts are available.
// ─────────────────────────────────────────────────────────────────────────────

namespace kibisis
{

    // Initialise PWM outputs and register the I2C slave handler.
    // i2c_init() must have been called on I2C_INSTANCE before this.
    void pwm_converter_init();

    // Apply any PWM target updates queued by the I2C ISR.
    // Call once per main loop iteration.
    void pwm_converter_apply_updates();

    // Update the encoder counts that will be returned to the Pi 5 on read requests.
    // Call this from your encoder module with fresh ADC/quadrature readings.
    void pwm_converter_set_encoder(uint16_t left_count, uint16_t right_count);

}  // namespace kibisis

#pragma once

#include <cstdint>

// ─────────────────────────────────────────────────────────────────────────────
// kibisis_encoder.hpp
//
// PIO-based quadrature encoder reader for two Pololu 37D motors.
//
// Each encoder produces 64 CPR on the motor shaft (counting both edges of
// both channels). Actual counts per output shaft revolution = 64 × gear_ratio.
//
// Two PIO state machines count pulses entirely in hardware. The CPU drains
// the FIFOs in the main loop and accumulates signed 32-bit counters.
//
// Usage:
//   1. Call kibisis::encoder_init() once at startup.
//   2. Call kibisis::encoder_update() every main loop iteration – this drains
//      the PIO FIFOs, updates the running counts, and forwards the latest
//      values to kibisis::pwm_converter_set_encoder() for I2C readback.
//   3. Optionally read counts directly with encoder_get_left/right().
// ─────────────────────────────────────────────────────────────────────────────

namespace kibisis {

// Initialise the PIO state machines for both encoders.
// Must be called before encoder_update().
void encoder_init();

void encoder_init();

// Drain PIO FIFOs, update running counts, and push latest values to
// pwm_converter_set_encoder() so the Pi 5 can read them back over I2C.
// Call once per main loop iteration.
void encoder_update();

// Direct count accessors (positive = forward, negative = backward).
// Sign depends on physical motor wiring – flip ENCODER_x_REVERSED in
// config.hpp if a motor counts backwards.
int32_t encoder_get_left();
int32_t encoder_get_right();

} // namespace kibisis

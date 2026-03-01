#pragma once

#include "hardware/i2c.h"

// ─────────────────────────────────────────────────────────────────────────────
// config.hpp
//
// Single source of truth for all hardware pin assignments, I2C addressing,
// register map, and PWM timing constants.
//
// Every other module includes this file – change a value here and it
// propagates everywhere.
// ─────────────────────────────────────────────────────────────────────────────

// ── I2C ──────────────────────────────────────────────────────────────────────

inline constexpr i2c_inst_t* I2C_INSTANCE = i2c0;
inline constexpr uint I2C_SDA_PIN = 4;
inline constexpr uint I2C_SCL_PIN = 5;
inline constexpr uint8_t I2C_SLAVE_ADDR = 0x01;  // _PICO_ADDRESS in kibisis_system.hpp
inline constexpr uint I2C_BAUDRATE = 100'000;    // raise to 400'000 for fast-mode

// ── Register map (must match kibisis_system.hpp) ──────────────────────────────

// Motor write transactions: [function][pwm_hi][pwm_lo]
// function byte encodes joint + direction:
inline constexpr uint8_t REG_LEFT_FORWARD = 0x00;
inline constexpr uint8_t REG_LEFT_BACKWARD = 0x01;
inline constexpr uint8_t REG_RIGHT_FORWARD = 0x02;
inline constexpr uint8_t REG_RIGHT_BACKWARD = 0x03;
inline constexpr uint8_t PWM_CHANNEL_COUNT = 4;

// Encoder read register addresses
inline constexpr uint8_t REG_ENCODER_LEFT = 0x04;   // _READ_ENCODER_LEFT  in kibisis_system.hpp
inline constexpr uint8_t REG_ENCODER_RIGHT = 0x05;  // _READ_ENCODER_RIGHT in kibisis_system.hpp

// ── PWM output pins (indexed by function byte 0x00–0x03) ─────────────────────

inline constexpr uint PWM_PIN_LEFT_FORWARD = 6;
inline constexpr uint PWM_PIN_LEFT_BACKWARD = 7;
inline constexpr uint PWM_PIN_RIGHT_FORWARD = 8;
inline constexpr uint PWM_PIN_RIGHT_BACKWARD = 9;

// ── PWM timing ────────────────────────────────────────────────────────────────
//
// Pico 2 system clock = 150 MHz
//   PWM base clock = 150 MHz / PWM_CLOCK_DIV
//   PWM frequency  = base clock / (PWM_WRAP + 1)
//
// Current: div=6 → 25 MHz base → wrap=24999 → 1 kHz
//
// For 50 Hz (servo/ESC): set PWM_CLOCK_DIV=120, PWM_WRAP=24999
//   150 MHz / 120 / 25000 = 50 Hz

inline constexpr uint PWM_CLOCK_DIV = 6;
inline constexpr uint16_t PWM_WRAP = 24'999;

// ── Encoder pins ─────────────────────────────────────────────────────────────
//
// Each encoder needs two pins (A and B channels).
// Pins must be consecutive and A must be even-numbered: the PIO quadrature
// program uses a pin pair, and the SDK's pio_add_program_at_offset requires
// the base pin to be even for the 2-pin IN mapping to work correctly.
//
// GP10/11 = left encoder  A/B
// GP12/13 = right encoder A/B

inline constexpr uint ENCODER_LEFT_PIN_A = 10;   // B is implicitly A+1 = 11
inline constexpr uint ENCODER_RIGHT_PIN_A = 12;  // B is implicitly A+1 = 13

// Encoder resolution: 64 counts per revolution of the motor shaft,
// counting both edges of both channels (quadrature, 4x).
// Multiply by your gearbox ratio to get counts per output shaft revolution.
// e.g. 30:1 gearbox → 64 × 30 = 1920 counts/rev at output.
// Update GEAR_RATIO to match your specific motor variant.
inline constexpr float ENCODER_CPR = 64.0f;
inline constexpr float GEAR_RATIO = 30.0f;  // ← set to your gear ratio
inline constexpr float COUNTS_PER_OUTPUT_REV = ENCODER_CPR * GEAR_RATIO;

// ── Encoder pins ──────────────────────────────────────────────────────────────
//
// Each encoder uses two consecutive GPIOs (A, then B = A+1).
// Chosen to follow the PWM outputs (GP6–GP9) with no gaps.
//
// Wiring (Pololu 37D colour code):
//   Yellow → A pin   White → B pin
//   Blue   → 3.3 V   Green → GND

inline constexpr uint ENCODER_LEFT_PIN_A = 10;   // B = GP11
inline constexpr uint ENCODER_RIGHT_PIN_A = 12;  // B = GP13

// Set to true if a motor's count goes negative when driving forward.
// Corrects for reversed encoder wiring without needing to swap physical wires.
inline constexpr bool ENCODER_LEFT_REVERSED = false;
inline constexpr bool ENCODER_RIGHT_REVERSED = false;

// Motor encoder resolution: 64 counts per motor shaft revolution,
// counting both edges of both channels (quadrature, 4× decoding).
// Multiply by your gearbox ratio to get counts per output shaft revolution.
// e.g. 30:1 gearbox → 64 × 30 = 1920 counts/rev at the wheel.
inline constexpr int32_t ENCODER_CPR = 64;

// ── Watchdog ──────────────────────────────────────────────────────────────────

inline constexpr uint32_t WATCHDOG_TIMEOUT_MS = 100;
inline constexpr bool WATCHDOG_PAUSE_DEBUG = true;

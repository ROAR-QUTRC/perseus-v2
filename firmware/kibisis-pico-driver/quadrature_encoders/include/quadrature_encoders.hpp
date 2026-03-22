#pragma once

#include <cstdint>

#include "hardware/pio.h"

#include "hardware/pio.h"

/// Quadrature encoder reader for Kibisis drive motors.
///
/// Channel A and B pins must be consecutive GPIOs (B = A + 1).
/// Both encoders share pio0, using state machines 0 and 1.
/// pio1 is reserved for the WS2812 status light.
///
/// Motor spec: Pololu 37D, 100:1 gearbox, 64 CPR motor shaft.
/// Full resolution: 64 × 4 × 100 = 25 600 counts/rev output shaft.
class QuadratureEncoders
{
public:
    QuadratureEncoders() = default;

    /// Load PIO program and start both state machines.
    /// Must be called once before getCountA() / getCountB().
    void init();

    /// Read current raw count for encoder A.
    /// Positive = forward, negative = reverse.
    [[nodiscard]] int32_t getCountA() const;

    /// Read current raw count for encoder B.
    [[nodiscard]] int32_t getCountB() const;

    /// Reset both counts to zero by re-initialising the state machines.
    void resetCounts();

private:
    // Encoder A: GP13 (ch A), GP14 (ch B)
    static constexpr uint kEncAPinA = 13;
    static constexpr uint kEncAPinB = 14;

    // Encoder B: GP15 (ch A), GP16 (ch B)
    static constexpr uint kEncBPinA = 15;
    static constexpr uint kEncBPinB = 16;

    static constexpr uint kEncASm   = 0;
    static constexpr uint kEncBSm   = 1;

    uint pio_program_offset_ = 0;
    bool initialised_ = false;
};
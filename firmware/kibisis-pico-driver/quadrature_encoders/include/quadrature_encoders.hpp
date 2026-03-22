#pragma once
#include "hardware/pio.h"
#include <cstdint>

// Pin assignments
// Channel A and B must be consecutive GPIOs (B = A + 1)
static constexpr uint ENC_A_PIN_A = 13;  // Encoder A: Channel A
static constexpr uint ENC_A_PIN_B = 14;  // Encoder A: Channel B  (must be A+1)
static constexpr uint ENC_B_PIN_A = 15;  // Encoder B: Channel A
static constexpr uint ENC_B_PIN_B = 16;  // Encoder B: Channel B  (must be A+1)

// Both encoders share one PIO block (pio0), using state machines 0 and 1.
// pio1 is left free for future use.
#define ENC_PIO pio0
static constexpr uint ENC_A_SM    = 0;
static constexpr uint ENC_B_SM    = 1;

// Motor spec (Pololu 37D, 100:1 gearbox, 64 CPR motor shaft)
// Full resolution: 64 CPR * 4 edges * 100:1 = 25,600 counts/rev output shaft
// int32_t range: \u00b12,147,483,648 counts \u2192 \u00b183,886 output shaft revolutions
// At 100 RPM no-load: ~10,667 counts/sec \u2192 int32_t lasts ~55 hours continuous

class QuadratureEncoders {
public:
    QuadratureEncoders() = default;

    // Load PIO program and start both state machines.
    // Must be called once before getCountA() / getCountB().
    void init();

    // Read current raw count for encoder A.
    // Positive = forward direction, negative = reverse.
    // Thread-safe: briefly stalls the PIO SM to push X, then reads FIFO.
    int32_t getCountA() const;

    // Read current raw count for encoder B.
    int32_t getCountB() const;

    // Reset both counts to zero by re-initialising the state machines.
    // Use sparingly \u2014 stalls both SMs briefly.
    void resetCounts();

private:
    uint pio_program_offset_ = 0;
    bool initialised_        = false;
};

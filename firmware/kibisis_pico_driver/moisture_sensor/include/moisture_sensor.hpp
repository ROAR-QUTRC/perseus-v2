#pragma once
#include "pico/stdlib.h"
#include <cstdint>

// Moisture sensor wired to GP26 (ADC0)
// Pico ADC is 12-bit: 0–4095
static constexpr uint MOISTURE_ADC_PIN     = 26;
static constexpr uint MOISTURE_ADC_CHANNEL = 0;

class MoistureSensor {
public:
    MoistureSensor() = default;

    // Initialise ADC and configure pin.
    // Must be called once before sample().
    void init();

    // Take a single ADC reading and store it internally.
    // Called by the main loop when the Pi has set the MOISTURE_SAMPLE register.
    void sample();

    // Return the last sampled value (0–4095).
    // Returns 0 if sample() has never been called.
    uint16_t getValue() const { return value_; }

private:
    uint16_t value_ = 0;
};
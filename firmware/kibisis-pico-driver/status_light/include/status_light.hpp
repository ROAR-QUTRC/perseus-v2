#pragma once

#include <cstdint>

#include "hardware/pio.h"
#include "pico/stdlib.h"

/// WS2812 status LED driver for the GlowBit 1×8 module.
///
/// Drives 8 WS2812 LEDs solid amber via PIO.
/// Uses pio1 SM0 to avoid conflict with quadrature encoders on pio0.
/// Data pin: GP28.
class StatusLight
{
public:
    void init();
    void setAmber();

private:
    static constexpr uint kLedPin   = 28;
    static constexpr uint kLedCount = 8;
    static constexpr float kFreqHz  = 800'000.0f;
    static constexpr bool kIsRgbw   = false;

    // Amber in GRB byte order: G=120, R=255, B=0
    static constexpr uint32_t kAmber =
        (static_cast<uint32_t>(120) << 16) |
        (static_cast<uint32_t>(255) <<  8) |
        static_cast<uint32_t>(0);

    PIO  pio_ = pio1;
    uint sm_  = 0;

    void put(uint32_t grb);
};
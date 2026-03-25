#pragma once
#include <cstdint>

#include "hardware/pio.h"
#include "pico/stdlib.h"

// WS2812 status LED driver for the GlowBit 1x8 module.
//
// Drives 8 WS2812 LEDs solid amber via PIO.
// Uses pio1 SM0 to avoid conflict with quadrature encoders on pio0.
// Data pin: GP28

class StatusLight
{
public:
    void init();
    void setAmber();

private:
    static constexpr uint LED_PIN = 28;
    static constexpr uint LED_COUNT = 8;
    static constexpr float FREQ_HZ = 800000.0f;
    static constexpr bool IS_RGBW = false;

    PIO pio_ = pio1;
    uint sm_ = 0;

    void put(uint32_t rgb);

    // In GRB format
    static constexpr uint32_t AMBER = (static_cast<uint32_t>(120) << 16) |  // g
                                      (static_cast<uint32_t>(255) << 8) |   // r
                                      static_cast<uint32_t>(0);             // b
};
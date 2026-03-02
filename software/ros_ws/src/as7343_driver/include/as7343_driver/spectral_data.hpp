#pragma once

#include <chrono>
#include <cstdint>

namespace as7343_driver
{

    struct SpectralReading
    {
        std::chrono::steady_clock::time_point timestamp;

        // 14 unique spectral channels (raw 16-bit ADC counts)
        uint16_t f1_405nm{0};   // Violet
        uint16_t f2_425nm{0};   // Violet-Blue
        uint16_t fz_450nm{0};   // Blue (wide)
        uint16_t f3_475nm{0};   // Blue-Cyan
        uint16_t f4_515nm{0};   // Green-Cyan
        uint16_t f5_550nm{0};   // Green
        uint16_t fy_555nm{0};   // Yellow-Green (wide)
        uint16_t fxl_600nm{0};  // Orange (wide)
        uint16_t f6_640nm{0};   // Red
        uint16_t f7_690nm{0};   // Deep Red
        uint16_t f8_745nm{0};   // Near-IR
        uint16_t nir_855nm{0};  // Near-IR

        // Broadband / auxiliary (averaged across 3 cycles)
        uint16_t vis_clear{0};
        uint16_t fd_flicker{0};

        // Status flags
        bool analog_saturation{false};
        bool digital_saturation{false};
        bool data_valid{false};
    };

    struct FlickerReading
    {
        std::chrono::steady_clock::time_point timestamp;

        uint16_t detected_frequency_hz{0};
        bool hz_100_valid{false};
        bool hz_120_valid{false};
        bool hz_100_detected{false};
        bool hz_120_detected{false};
        bool fd_saturation{false};
        bool fd_valid{false};
    };

}  // namespace as7343_driver

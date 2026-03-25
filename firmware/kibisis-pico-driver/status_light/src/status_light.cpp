#include "status_light.hpp"

#include "ws2812.pio.h"

void StatusLight::init()
{
    const uint offset = pio_add_program(pio_, &ws2812_program);
    ws2812_program_init(pio_, sm_, offset, kLedPin, kFreqHz, kIsRgbw);
    setAmber();
}

void StatusLight::setAmber()
{
    for (uint i = 0; i < kLedCount; ++i)
        put(kAmber);
}

void StatusLight::put(const uint32_t grb)
{
    pio_sm_put_blocking(pio_, sm_, grb << 8u);
}
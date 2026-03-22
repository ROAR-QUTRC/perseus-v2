#include "status-light.hpp"

#include "ws2812.pio.h"

void StatusLight::init()
{
    uint offset = pio_add_program(pio_, &ws2812_program);
    ws2812_program_init(pio_, sm_, offset, LED_PIN, FREQ_HZ, IS_RGBW);
    setAmber();
}

void StatusLight::setAmber()
{
    for (uint i = 0; i < LED_COUNT; ++i)
        put(AMBER);
}

void StatusLight::put(uint32_t rgb)
{
    pio_sm_put_blocking(pio_, sm_, rgb << 8u);
}

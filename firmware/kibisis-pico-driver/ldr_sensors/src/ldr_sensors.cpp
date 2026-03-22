#include "ldr_sensors.hpp"

void LdrSensors::init()
{
    gpio_init(kPinLdrAEnable);
    gpio_set_dir(kPinLdrAEnable, GPIO_OUT);
    gpio_put(kPinLdrAEnable, 0);

    gpio_init(kPinLdrBEnable);
    gpio_set_dir(kPinLdrBEnable, GPIO_OUT);
    gpio_put(kPinLdrBEnable, 0);

    adc_init();
    adc_gpio_init(26);  // ADC0 — LDR A
    adc_gpio_init(27);  // ADC1 — LDR B
}

LdrSensors::Reading LdrSensors::sample()
{
    Reading result{};

    // Ambient readings (LEDs off)
    adc_select_input(kAdcChLdrA);
    result.ldrAAmbient = adc_read();

    adc_select_input(kAdcChLdrB);
    result.ldrBAmbient = adc_read();

    // Illuminated readings (LEDs on)
    gpio_put(kPinLdrAEnable, 1);
    gpio_put(kPinLdrBEnable, 1);
    sleep_ms(kSettleMs);

    adc_select_input(kAdcChLdrA);
    result.ldrAIlluminated = adc_read();

    adc_select_input(kAdcChLdrB);
    result.ldrBIlluminated = adc_read();

    gpio_put(kPinLdrAEnable, 0);
    gpio_put(kPinLdrBEnable, 0);

    return result;
}
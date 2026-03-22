#include "ldr_sensors.hpp"

void LdrSensors::init()
{
    // Power enable pins — off by default
    gpio_init(PIN_LDR_A_EN);
    gpio_set_dir(PIN_LDR_A_EN, GPIO_OUT);
    gpio_put(PIN_LDR_A_EN, 0);

    gpio_init(PIN_LDR_B_EN);
    gpio_set_dir(PIN_LDR_B_EN, GPIO_OUT);
    gpio_put(PIN_LDR_B_EN, 0);

    // ADC
    adc_init();
    adc_gpio_init(26);  // ADC0 — LDR A
    adc_gpio_init(27);  // ADC1 — LDR B
}

LdrSensors::Reading LdrSensors::sample()
{
    Reading result = {};

    // --- Ambient readings (LEDs off) ---
    adc_select_input(ADC_CH_LDR_A);
    result.ldr_a_ambient = adc_read();

    adc_select_input(ADC_CH_LDR_B);
    result.ldr_b_ambient = adc_read();

    // --- Illuminated readings (LEDs on) ---
    gpio_put(PIN_LDR_A_EN, 1);
    gpio_put(PIN_LDR_B_EN, 1);
    sleep_ms(SETTLE_MS);

    adc_select_input(ADC_CH_LDR_A);
    result.ldr_a_illuminated = adc_read();

    adc_select_input(ADC_CH_LDR_B);
    result.ldr_b_illuminated = adc_read();

    // LEDs off
    gpio_put(PIN_LDR_A_EN, 0);
    gpio_put(PIN_LDR_B_EN, 0);

    return result;
}
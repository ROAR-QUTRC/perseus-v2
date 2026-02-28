#include "current_sensor.hpp"

#include <cstdio>

#include "hardware/adc.h"
#include "pico/stdlib.h"

namespace current_sensor {

// Z6428 sensor characteristics
constexpr float VCC = 3.3f;
constexpr float V_ZERO_CURRENT = VCC / 2.0f;  // Vcc/2 at 0A
constexpr float SENSITIVITY_V_PER_A = 0.185f;  // ~185mV/A

// ADC characteristics (12-bit, 3.3V reference)
constexpr float ADC_VREF = 3.3f;
constexpr float ADC_MAX = 4095.0f;

CurrentSensor::CurrentSensor(uint8_t adc_pin)
    : _adc_pin(adc_pin), _adc_input(adc_pin - 26)
{
}

void CurrentSensor::init()
{
    adc_init();
    adc_gpio_init(_adc_pin);
    printf("Current sensor initialized on ADC pin %u (input %u)\n", _adc_pin,
           _adc_input);
}

uint16_t CurrentSensor::read_current_ma()
{
    adc_select_input(_adc_input);
    uint16_t raw = adc_read();

    float voltage = (static_cast<float>(raw) / ADC_MAX) * ADC_VREF;
    float current_a = (voltage - V_ZERO_CURRENT) / SENSITIVITY_V_PER_A;

    // Take absolute value (bidirectional sensor)
    if (current_a < 0.0f) {
        current_a = -current_a;
    }

    return static_cast<uint16_t>(current_a * 1000.0f);
}

float CurrentSensor::read_current_amps()
{
    return read_current_ma() / 1000.0f;
}

}  // namespace current_sensor

#include "moisture_sensor.hpp"
#include "hardware/adc.h"

void MoistureSensor::init() {
    adc_init();
    adc_gpio_init(MOISTURE_ADC_PIN);
    adc_select_input(MOISTURE_ADC_CHANNEL);
}

void MoistureSensor::sample() {
    adc_select_input(MOISTURE_ADC_CHANNEL);
    value_ = adc_read();
}

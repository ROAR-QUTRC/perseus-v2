#pragma once

// core libs
#include <driver/gpio.h>

// standard libs
#include <cstdint>

// rover libs
#include <rover_io.hpp>

#define ROVER_ADC_DISABLE_ERROR  5000
#define ROVER_ADC_ERR_RETURN_VAL 10000

// macros
#define ROVER_ADC_DIVIDER_TO_SOURCE_VOLTAGE(_voltage, _r1, _r2) ((_voltage * (_r1 + _r2)) / _r2)

// configureChannel will initialise the ADC when needed
void adc_set_channel_enabled(gpio_num_t gpio, bool enable, bool delayedInit = false);
void adc_set_error_voltage(gpio_num_t gpio, uint16_t errorVtg);
void adc_deinitialize();

int32_t adc_get_voltage(gpio_num_t gpio);
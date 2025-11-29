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
void adcSetChannelEnabled(gpio_num_t gpio, bool enable, bool delayedInit = false);
void adcSetErrorVoltage(gpio_num_t gpio, uint16_t errorVtg);
void adcDeinit();

bool adcHasNewReading(gpio_num_t gpio);
int32_t adcGetVoltage(gpio_num_t gpio);
#pragma once

#include <cstdint>

namespace current_sensor {

/// Z6428 Hall-effect current sensor on ADC pin
class CurrentSensor {
public:
    explicit CurrentSensor(uint8_t adc_pin = 26);

    /// Initialize ADC
    void init();

    /// Read current in milliamps
    uint16_t read_current_ma();

    /// Read current in amps
    float read_current_amps();

private:
    uint8_t _adc_pin;
    uint8_t _adc_input;
};

}  // namespace current_sensor

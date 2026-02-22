#pragma once

#include <cstdint>

namespace ina228_driver
{

    // INA228 Register Addresses
    constexpr uint8_t REG_CONFIG = 0x00;
    constexpr uint8_t REG_ADC_CONFIG = 0x01;
    constexpr uint8_t REG_SHUNT_CAL = 0x02;
    constexpr uint8_t REG_SHUNT_TEMPCO = 0x03;
    constexpr uint8_t REG_VSHUNT = 0x04;
    constexpr uint8_t REG_VBUS = 0x05;
    constexpr uint8_t REG_DIETEMP = 0x06;
    constexpr uint8_t REG_CURRENT = 0x07;
    constexpr uint8_t REG_POWER = 0x08;
    constexpr uint8_t REG_ENERGY = 0x09;
    constexpr uint8_t REG_CHARGE = 0x0A;
    constexpr uint8_t REG_DIAG_ALRT = 0x0B;
    constexpr uint8_t REG_MANUFACTURER_ID = 0x3E;
    constexpr uint8_t REG_DEVICE_ID = 0x3F;

    // CONFIG register bits
    constexpr uint16_t CONFIG_RST = 0x8000;
    constexpr uint16_t CONFIG_RSTACC = 0x4000;
    constexpr uint16_t CONFIG_ADCRANGE = 0x0010;

    // Expected identification values
    constexpr uint16_t MANUFACTURER_ID = 0x5449;   // "TI"
    constexpr uint16_t DEVICE_ID = 0x2280;          // INA228 (bits [15:4])
    constexpr uint16_t DEVICE_ID_MASK = 0xFFF0;     // Mask out revision bits [3:0]

    // Conversion constants (ADCRANGE = 0, the default)
    constexpr double VBUS_LSB = 195.3125e-6;        // 195.3125 uV/LSB
    constexpr double VSHUNT_LSB = 312.5e-9;          // 312.5 nV/LSB
    constexpr double DIETEMP_LSB = 7.8125e-3;        // 7.8125 m°C/LSB
    constexpr double POWER_SCALE = 3.2;               // POWER_LSB = 3.2 * CURRENT_LSB
    constexpr double ENERGY_SCALE = 16.0 * 3.2;       // ENERGY_LSB = 16 * 3.2 * CURRENT_LSB
    constexpr double SHUNT_CAL_FACTOR = 13107.2e6;    // Calibration register constant

}  // namespace ina228_driver

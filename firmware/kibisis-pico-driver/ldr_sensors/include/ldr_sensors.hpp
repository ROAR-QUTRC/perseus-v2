#pragma once

#include <cstdint>

#include "hardware/adc.h"
#include "pico/stdlib.h"

/// LDR sensor module for Kibisis.
///
/// Two LDRs on ADC0 (GP26) and ADC1 (GP27).
/// Each LDR LED is powered via a dedicated GPIO enable pin:
///   LDR A → ADC0 (GP26), LED powered by GP20
///   LDR B → ADC1 (GP27), LED powered by GP21
///
/// sample() takes four readings:
///   1. LDR A ambient (LEDs off)
///   2. LDR B ambient (LEDs off)
///   3. LDR A illuminated (LEDs on)
///   4. LDR B illuminated (LEDs on)
///
/// All readings are 12-bit (0–4095).
class LdrSensors
{
public:
    struct Reading
    {
        uint16_t ldrAAmbient;      ///< LDR A reading before LEDs on
        uint16_t ldrBAmbient;      ///< LDR B reading before LEDs on
        uint16_t ldrAIlluminated;  ///< LDR A reading with LEDs on
        uint16_t ldrBIlluminated;  ///< LDR B reading with LEDs on
    };

    void init();

    /// Power LEDs, take ambient and illuminated readings, power LEDs off.
    [[nodiscard]] Reading sample();

private:
    static constexpr uint kPinLdrAEnable = 20;  ///< GPIO enabling LDR A LED power
    static constexpr uint kPinLdrBEnable = 21;  ///< GPIO enabling LDR B LED power
    static constexpr uint kAdcChLdrA     = 0;   ///< GP26 = ADC channel 0
    static constexpr uint kAdcChLdrB     = 1;   ///< GP27 = ADC channel 1
    static constexpr uint kSettleMs      = 10;  ///< LED stabilisation time in ms
};
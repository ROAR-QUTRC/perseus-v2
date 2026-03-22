#pragma once
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <cstdint>

// LDR sensor module for Kibisis.
//
// Two LDRs on ADC0 (GP26) and ADC1 (GP27).
// Each LDR is powered via a dedicated GPIO enable pin:
//   LDR A -> ADC0 (GP26), powered by GP20
//   LDR B -> ADC1 (GP27), powered by GP21
//
// sample() takes four readings:
//   1. LDR A ambient (LEDs off)
//   2. LDR B ambient (LEDs off)
//   3. LDR A illuminated (LEDs on)
//   4. LDR B illuminated (LEDs on)
//
// All readings are 12-bit (0–4095).

class LdrSensors
{
public:
    struct Reading {
        uint16_t ldr_a_ambient;      // LDR A before LEDs on
        uint16_t ldr_b_ambient;      // LDR B before LEDs on
        uint16_t ldr_a_illuminated;  // LDR A with LEDs on
        uint16_t ldr_b_illuminated;  // LDR B with LEDs on
    };

    void init();
    Reading sample();

private:
    static constexpr uint PIN_LDR_A_EN = 20;   // power enable for LDR A LED
    static constexpr uint PIN_LDR_B_EN = 21;   // power enable for LDR B LED
    static constexpr uint ADC_CH_LDR_A = 0;    // GP26 = ADC0
    static constexpr uint ADC_CH_LDR_B = 1;    // GP27 = ADC1
    static constexpr uint SETTLE_MS    = 10;   // time for LDR to stabilise after LED on
};
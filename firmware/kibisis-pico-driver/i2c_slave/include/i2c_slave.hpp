#pragma once

#include <cstdint>
#include <cstring>

#include "hardware/i2c.h"
#include "pico/stdlib.h"

/// Register map constants shared between Pico firmware and Pi driver.
namespace kibisis
{
// I2C slave address
constexpr uint8_t kSlaveAddr             = 0x41;

// Pi→Pico command registers
constexpr uint8_t kRegMotorASpeed        = 0x00;
constexpr uint8_t kRegMotorBSpeed        = 0x01;
constexpr uint8_t kRegSpaceMotorSpeed    = 0x02;

// Pico→Pi state registers
constexpr uint8_t kRegStatus             = 0x20;

// Encoder counts 0x10–0x17 (int32_t LE, A then B)
constexpr uint8_t kRegEncACount0         = 0x10;
constexpr uint8_t kRegEncBCount0         = 0x14;

// LDR trigger (Pi→Pico)
constexpr uint8_t kRegLdrSample          = 0x40;

// LDR results (Pico→Pi, uint16_t LE each)
constexpr uint8_t kRegLdrAAmbient0       = 0x41;
constexpr uint8_t kRegLdrBAmbient0       = 0x43;
constexpr uint8_t kRegLdrAIlluminated0   = 0x45;
constexpr uint8_t kRegLdrBIlluminated0   = 0x47;

constexpr size_t  kRegCount              = 73;  // 0x00–0x48
}  // namespace kibisis

/// I2C slave driver for the Kibisis Pico.
///
/// Owns the I2C0 peripheral in slave mode at address kibisis::kSlaveAddr.
/// Pi writes command registers; Pico writes state registers into shadow_regs_
/// and atomically publishes them via commitRegisters().
class I2CSlave
{
public:
    void init();

    // ----- Getters (Pi→Pico command registers) -----
    [[nodiscard]] int8_t getMotorASpeed()     const;
    [[nodiscard]] int8_t getMotorBSpeed()     const;
    [[nodiscard]] int8_t getSpaceMotorSpeed() const;

    /// Returns true (and clears the flag) if the Pi has requested an LDR sample.
    [[nodiscard]] bool getLdrSampleTrigger();

    // ----- Setters (Pico→Pi state, written to shadow then committed) -----
    void setStatus(uint8_t status);
    void setEncoderA(int32_t counts);
    void setEncoderB(int32_t counts);
    void setLdrAmbientA(uint16_t value);
    void setLdrAmbientB(uint16_t value);
    void setLdrIlluminatedA(uint16_t value);
    void setLdrIlluminatedB(uint16_t value);

    /// Atomically copy shadow→live for all Pico→Pi registers.
    /// Call once per main-loop iteration after updating all shadow values.
    void commitRegisters();

private:
    static void i2cIrqHandler();
    void handleIrq();

    static constexpr uint kPinSda = 4;
    static constexpr uint kPinScl = 5;

    volatile uint8_t live_regs_[kibisis::kRegCount]   = {};
    volatile uint8_t shadow_regs_[kibisis::kRegCount] = {};
    volatile uint8_t reg_ptr_ = 0xFF;

    static I2CSlave* instance_;
};
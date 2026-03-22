#pragma once
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <cstdint>
#include <cstring>

static constexpr uint8_t SLAVE_ADDR                  = 0x41;

// Pi→Pico (command) registers
static constexpr uint8_t REG_MOTOR_A_SPEED           = 0x00;
static constexpr uint8_t REG_MOTOR_B_SPEED           = 0x01;
static constexpr uint8_t REG_SPACE_MOTOR_SPEED        = 0x02;

// Pico→Pi (state) registers
static constexpr uint8_t REG_STATUS                  = 0x20;

// Encoder counts (0x10–0x17, int32_t LE per encoder)
// 0x10–0x13 = Encoder A, 0x14–0x17 = Encoder B

// LDR trigger (Pi→Pico)
static constexpr uint8_t REG_LDR_SAMPLE              = 0x40;

// LDR ambient readings — LEDs off (Pico→Pi, uint16_t LE)
static constexpr uint8_t REG_LDR_A_AMBIENT_0         = 0x41;
static constexpr uint8_t REG_LDR_A_AMBIENT_1         = 0x42;
static constexpr uint8_t REG_LDR_B_AMBIENT_0         = 0x43;
static constexpr uint8_t REG_LDR_B_AMBIENT_1         = 0x44;

// LDR illuminated readings — LEDs on (Pico→Pi, uint16_t LE)
static constexpr uint8_t REG_LDR_A_ILLUMINATED_0     = 0x45;
static constexpr uint8_t REG_LDR_A_ILLUMINATED_1     = 0x46;
static constexpr uint8_t REG_LDR_B_ILLUMINATED_0     = 0x47;
static constexpr uint8_t REG_LDR_B_ILLUMINATED_1     = 0x48;

static constexpr size_t  REG_COUNT                   = 73;   // 0x00–0x48

class I2CSlave
{
public:
    void init();

    // Motor speed getters (Pi→Pico)
    int8_t getMotorASpeed() const { return static_cast<int8_t>(live_regs_[REG_MOTOR_A_SPEED]); }
    int8_t getMotorBSpeed() const { return static_cast<int8_t>(live_regs_[REG_MOTOR_B_SPEED]); }
    int8_t getSpaceMotorSpeed() const { return static_cast<int8_t>(live_regs_[REG_SPACE_MOTOR_SPEED]); }

    // Status setter (Pico→Pi)
    void setStatus(uint8_t v) { shadow_regs_[REG_STATUS] = v; }

    // Encoder setters (Pico→Pi)
    void setEncoderA(int32_t counts);
    void setEncoderB(int32_t counts);

    // LDR: returns true (and clears the flag) if Pi has requested a sample
    bool getLdrSampleTrigger();

    // LDR result setters (Pico→Pi)
    void setLdrAmbientA(uint16_t value);
    void setLdrAmbientB(uint16_t value);
    void setLdrIlluminatedA(uint16_t value);
    void setLdrIlluminatedB(uint16_t value);

    // Atomically commit shadow→live for all Pico→Pi registers
    void commitRegisters();

private:
    static void i2c_irq_handler();
    void handle_irq();

    volatile uint8_t live_regs_[REG_COUNT]   = {};
    volatile uint8_t shadow_regs_[REG_COUNT] = {};
    volatile uint8_t reg_ptr_ = 0xFF;

    static I2CSlave* instance_;
};
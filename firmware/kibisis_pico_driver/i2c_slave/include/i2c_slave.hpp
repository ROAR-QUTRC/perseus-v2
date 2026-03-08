#pragma once
#include "hardware/i2c.h"
#include <cstdint>

// I2C configuration
static constexpr uint    I2C_PORT_NUM = 0;
static constexpr uint    I2C_SDA_PIN  = 4;
static constexpr uint    I2C_SCL_PIN  = 5;
static constexpr uint    I2C_FREQ_HZ  = 100000;
static constexpr uint8_t SLAVE_ADDR   = 0x41;

// Register map
// Pi 5 writes motor speeds, reads encoder counts and status
namespace Reg {
    // Writable by Pi 5
    static constexpr uint8_t MOTOR_A_SPEED        = 0x00;  // int8_t: -100 to 100
    static constexpr uint8_t MOTOR_B_SPEED        = 0x01;  // int8_t: -100 to 100
    static constexpr uint8_t SPACE_MOTOR_SPEED    = 0x02;  // int8_t: -100 to 100

    // Readable by Pi 5 (int32_t, 4 bytes each, little-endian)
    static constexpr uint8_t ENC_A_COUNT_0 = 0x10;
    static constexpr uint8_t ENC_A_COUNT_1 = 0x11;
    static constexpr uint8_t ENC_A_COUNT_2 = 0x12;
    static constexpr uint8_t ENC_A_COUNT_3 = 0x13;
    static constexpr uint8_t ENC_B_COUNT_0 = 0x14;
    static constexpr uint8_t ENC_B_COUNT_1 = 0x15;
    static constexpr uint8_t ENC_B_COUNT_2 = 0x16;
    static constexpr uint8_t ENC_B_COUNT_3 = 0x17;

    static constexpr uint8_t STATUS        = 0x20;  // status/heartbeat byte
    static constexpr uint8_t REG_COUNT     = 0x21;
}

class I2CSlave {
public:
    I2CSlave();
    void init();

    // Called from main loop to write encoder counts and status into the
    // shadow buffer, then atomically commit them to the live registers.
    // Safe to call at any time - interrupts are briefly disabled during commit.
    void setEncoderA(int32_t count);
    void setEncoderB(int32_t count);
    void setStatus(uint8_t status);
    void commitRegisters();  // atomically copies shadow -> live

    // Called from main loop to retrieve what Pi 5 wrote
    int8_t getMotorASpeed() const;
    int8_t getMotorBSpeed() const;
    int8_t getSpaceMotorSpeed() const;

private:
    static void      i2cIrqHandler();
    static I2CSlave* instance_;  // singleton for IRQ callback

    void handleIrq();

    // live_regs_ is read by the IRQ handler - never written outside commitRegisters()
    // shadow_regs_ is written freely by the main loop
    uint8_t live_regs_[Reg::REG_COUNT]   = {};
    uint8_t shadow_regs_[Reg::REG_COUNT] = {};

    uint8_t reg_addr_      = 0;
    bool    addr_received_ = false;
};

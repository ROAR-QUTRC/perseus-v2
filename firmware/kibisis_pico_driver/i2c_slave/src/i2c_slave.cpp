#include "i2c_slave.hpp"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/structs/i2c.h"
#include "hardware/sync.h"
#include <cstring>

I2CSlave* I2CSlave::instance_ = nullptr;

I2CSlave::I2CSlave() {
    instance_ = this;
}

void I2CSlave::init() {
    i2c_init(i2c0, I2C_FREQ_HZ);

    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Put I2C0 into slave mode at our address
    i2c_set_slave_mode(i2c0, true, SLAVE_ADDR);

    // Enable interrupts we care about:
    //   RX_FULL  - master has written a byte to us
    //   RD_REQ   - master is requesting a byte from us
    //   TX_ABRT  - master aborted a transaction
    //   STOP_DET - transaction complete
    i2c0->hw->intr_mask = I2C_IC_INTR_MASK_M_RX_FULL_BITS  |
                          I2C_IC_INTR_MASK_M_RD_REQ_BITS   |
                          I2C_IC_INTR_MASK_M_TX_ABRT_BITS  |
                          I2C_IC_INTR_MASK_M_STOP_DET_BITS;

    irq_set_exclusive_handler(I2C0_IRQ, i2cIrqHandler);
    irq_set_enabled(I2C0_IRQ, true);
}

void I2CSlave::i2cIrqHandler() {
    if (instance_) instance_->handleIrq();
}

void I2CSlave::handleIrq() {
    uint32_t status = i2c0->hw->intr_stat;

    // Master aborted - reset state
    if (status & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
        i2c0->hw->clr_tx_abrt;
        addr_received_ = false;
    }

    // Master wrote a byte to us
    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        uint8_t byte = static_cast<uint8_t>(i2c0->hw->data_cmd);
        if (!addr_received_) {
            // First byte of a transaction is always the register address
            reg_addr_      = byte;
            addr_received_ = true;
        } else {
            // Subsequent bytes are data written to sequential registers
            // Written directly to live_regs_ since motor speeds are single bytes
            // (no multi-byte consistency issue)
            if (reg_addr_ < Reg::REG_COUNT) {
                live_regs_[reg_addr_] = byte;
                // Track last time a motor command register was written so the
                // main loop can detect Pi comms timeout
                if (reg_addr_ <= Reg::SPACE_MOTOR_SPEED) {
                    last_write_time_us_ = time_us_64();
                }
                reg_addr_++;
            }
        }
    }

    // Master is reading - serve from live_regs_ only
    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        if (reg_addr_ < Reg::REG_COUNT) {
            i2c0->hw->data_cmd = live_regs_[reg_addr_++];
        } else {
            i2c0->hw->data_cmd = 0xFF;  // out of range sentinel
        }
        i2c0->hw->clr_rd_req;
    }

    // Transaction complete - reset for next transaction
    if (status & I2C_IC_INTR_STAT_R_STOP_DET_BITS) {
        addr_received_ = false;
        i2c0->hw->clr_stop_det;
    }
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void I2CSlave::setEncoderA(int32_t count) {
    memcpy(&shadow_regs_[Reg::ENC_A_COUNT_0], &count, sizeof(count));
}

void I2CSlave::setEncoderB(int32_t count) {
    memcpy(&shadow_regs_[Reg::ENC_B_COUNT_0], &count, sizeof(count));
}

void I2CSlave::setMoistureValue(uint16_t value) {
    memcpy(&shadow_regs_[Reg::MOISTURE_VALUE_0], &value, sizeof(value));
}

bool I2CSlave::getMoistureSampleRequested() const {
    return live_regs_[Reg::MOISTURE_SAMPLE] != 0;
}

void I2CSlave::clearMoistureSample() {
    // Clear in both live and shadow so the Pi sees it reset on next read
    uint32_t saved = save_and_disable_interrupts();
    live_regs_[Reg::MOISTURE_SAMPLE]   = 0;
    shadow_regs_[Reg::MOISTURE_SAMPLE] = 0;
    restore_interrupts(saved);
}

void I2CSlave::setStatus(uint8_t status) {
    shadow_regs_[Reg::STATUS] = status;
}

void I2CSlave::commitRegisters() {
    // Briefly disable interrupts so the IRQ handler cannot fire mid-copy,
    // guaranteeing the Pi always reads a consistent snapshot of live_regs_.
    // Only copies the readable registers (encoders + status) from shadow to live.
    // The writable registers (motor speeds at 0x00-0x01) are written directly
    // by the IRQ and must not be overwritten here.
    uint32_t saved = save_and_disable_interrupts();
    memcpy(&live_regs_[Reg::ENC_A_COUNT_0],
           &shadow_regs_[Reg::ENC_A_COUNT_0],
           Reg::REG_COUNT - Reg::ENC_A_COUNT_0);
    restore_interrupts(saved);
}

bool I2CSlave::hasCommandTimedOut() const {
    // Never timed out if we haven't received even one command yet
    if (last_write_time_us_ == 0) return false;
    uint64_t elapsed_us = time_us_64() - last_write_time_us_;
    return elapsed_us > (static_cast<uint64_t>(COMMS_TIMEOUT_MS) * 1000ULL);
}

int8_t I2CSlave::getMotorASpeed() const {
    // Motor speed registers are written by the IRQ as single bytes so no
    // multi-byte consistency issue - read directly from live_regs_
    return static_cast<int8_t>(live_regs_[Reg::MOTOR_A_SPEED]);
}

int8_t I2CSlave::getMotorBSpeed() const {
    return static_cast<int8_t>(live_regs_[Reg::MOTOR_B_SPEED]);
}

int8_t I2CSlave::getSpaceMotorSpeed() const {
    return static_cast<int8_t>(live_regs_[Reg::SPACE_MOTOR_SPEED]);
}

#include "i2c_slave.hpp"
#include "hardware/irq.h"
#include "hardware/regs/i2c.h"
#include "hardware/sync.h"

I2CSlave* I2CSlave::instance_ = nullptr;

void I2CSlave::init()
{
    instance_ = this;
    i2c_init(i2c0, 100'000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);
    i2c0->hw->enable = 0;
    i2c0->hw->sar    = SLAVE_ADDR;
    i2c0->hw->con = I2C_IC_CON_IC_RESTART_EN_BITS
                    | I2C_IC_CON_TX_EMPTY_CTRL_BITS;
    i2c0->hw->enable = 1;
    i2c0->hw->intr_mask = I2C_IC_INTR_MASK_M_RX_FULL_BITS
                        | I2C_IC_INTR_MASK_M_RD_REQ_BITS
                        | I2C_IC_INTR_MASK_M_TX_ABRT_BITS
                        | I2C_IC_INTR_MASK_M_STOP_DET_BITS;
    irq_set_exclusive_handler(I2C0_IRQ, i2c_irq_handler);
    irq_set_enabled(I2C0_IRQ, true);
}

void I2CSlave::commitRegisters()
{
    uint32_t saved = save_and_disable_interrupts();

    // Status
    live_regs_[REG_STATUS] = shadow_regs_[REG_STATUS];

    // Encoder counts (0x10–0x17)
    memcpy(const_cast<uint8_t*>(&live_regs_[0x10]),
           const_cast<uint8_t*>(&shadow_regs_[0x10]), 8);

    // LDR results — ambient + illuminated (0x41–0x48, 8 bytes)
    memcpy(const_cast<uint8_t*>(&live_regs_[REG_LDR_A_AMBIENT_0]),
           const_cast<uint8_t*>(&shadow_regs_[REG_LDR_A_AMBIENT_0]), 8);

    restore_interrupts(saved);
}

void I2CSlave::i2c_irq_handler() { if (instance_) instance_->handle_irq(); }

void I2CSlave::handle_irq()
{
    uint32_t status = i2c0->hw->intr_stat;

    if (status & I2C_IC_INTR_STAT_R_TX_ABRT_BITS)
        (void)i2c0->hw->clr_tx_abrt;

    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        uint8_t byte = static_cast<uint8_t>(i2c0->hw->data_cmd & 0xFF);
        if (reg_ptr_ == 0xFF) {
            reg_ptr_ = byte < REG_COUNT ? byte : 0;
        } else {
            if (reg_ptr_ < REG_COUNT) live_regs_[reg_ptr_++] = byte;
        }
    }

    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        uint8_t val = reg_ptr_ < REG_COUNT ? live_regs_[reg_ptr_++] : 0;
        i2c0->hw->data_cmd = val;
        (void)i2c0->hw->clr_rd_req;
    }

    if (status & I2C_IC_INTR_STAT_R_STOP_DET_BITS) {
        reg_ptr_ = 0xFF;
        (void)i2c0->hw->clr_stop_det;
    }
}

void I2CSlave::setEncoderA(int32_t counts)
{
    memcpy(const_cast<uint8_t*>(&shadow_regs_[0x10]), &counts, 4);
}

void I2CSlave::setEncoderB(int32_t counts)
{
    memcpy(const_cast<uint8_t*>(&shadow_regs_[0x14]), &counts, 4);
}

bool I2CSlave::getLdrSampleTrigger()
{
    uint32_t saved = save_and_disable_interrupts();
    bool triggered = live_regs_[REG_LDR_SAMPLE] != 0;
    if (triggered) live_regs_[REG_LDR_SAMPLE] = 0;
    restore_interrupts(saved);
    return triggered;
}

void I2CSlave::setLdrAmbientA(uint16_t value)
{
    shadow_regs_[REG_LDR_A_AMBIENT_0] = value & 0xFF;
    shadow_regs_[REG_LDR_A_AMBIENT_1] = (value >> 8) & 0xFF;
}

void I2CSlave::setLdrAmbientB(uint16_t value)
{
    shadow_regs_[REG_LDR_B_AMBIENT_0] = value & 0xFF;
    shadow_regs_[REG_LDR_B_AMBIENT_1] = (value >> 8) & 0xFF;
}

void I2CSlave::setLdrIlluminatedA(uint16_t value)
{
    shadow_regs_[REG_LDR_A_ILLUMINATED_0] = value & 0xFF;
    shadow_regs_[REG_LDR_A_ILLUMINATED_1] = (value >> 8) & 0xFF;
}

void I2CSlave::setLdrIlluminatedB(uint16_t value)
{
    shadow_regs_[REG_LDR_B_ILLUMINATED_0] = value & 0xFF;
    shadow_regs_[REG_LDR_B_ILLUMINATED_1] = (value >> 8) & 0xFF;
}
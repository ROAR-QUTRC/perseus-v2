#include "i2c_slave.hpp"


#include "hardware/irq.h"
#include "hardware/regs/i2c.h"
#include "hardware/sync.h"

I2CSlave* I2CSlave::instance_ = nullptr;

void I2CSlave::init()
{
    instance_ = this;
    i2c_init(i2c0, 100'000);
    gpio_set_function(kPinSda, GPIO_FUNC_I2C);
    gpio_set_function(kPinScl, GPIO_FUNC_I2C);
    gpio_pull_up(kPinSda);
    gpio_pull_up(kPinScl);
    i2c0->hw->enable = 0;
    i2c0->hw->sar    = kibisis::kSlaveAddr;
    i2c0->hw->con    = I2C_IC_CON_IC_RESTART_EN_BITS | I2C_IC_CON_TX_EMPTY_CTRL_BITS;
    i2c0->hw->enable = 1;
    i2c0->hw->intr_mask = I2C_IC_INTR_MASK_M_RX_FULL_BITS
                        | I2C_IC_INTR_MASK_M_RD_REQ_BITS
                        | I2C_IC_INTR_MASK_M_TX_ABRT_BITS
                        | I2C_IC_INTR_MASK_M_STOP_DET_BITS;
    irq_set_exclusive_handler(I2C0_IRQ, i2cIrqHandler);
    irq_set_enabled(I2C0_IRQ, true);
}

void I2CSlave::commitRegisters()
{
    const uint32_t saved = save_and_disable_interrupts();

    live_regs_[kibisis::kRegStatus] = shadow_regs_[kibisis::kRegStatus];

    // Encoder counts (0x10–0x17, 8 bytes)
    memcpy(const_cast<uint8_t*>(&live_regs_[kibisis::kRegEncACount0]),
           const_cast<uint8_t*>(&shadow_regs_[kibisis::kRegEncACount0]), 8);

    // LDR results — ambient + illuminated (0x41–0x48, 8 bytes)
    memcpy(const_cast<uint8_t*>(&live_regs_[kibisis::kRegLdrAAmbient0]),
           const_cast<uint8_t*>(&shadow_regs_[kibisis::kRegLdrAAmbient0]), 8);

    restore_interrupts(saved);
}

void I2CSlave::i2cIrqHandler()
{
    if (instance_ != nullptr)
        instance_->handleIrq();
}

void I2CSlave::handleIrq()
{
    const uint32_t status = i2c0->hw->intr_stat;

    if (status & I2C_IC_INTR_STAT_R_TX_ABRT_BITS)
        (void)i2c0->hw->clr_tx_abrt;

    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS)
    {
        const uint8_t byte = static_cast<uint8_t>(i2c0->hw->data_cmd & 0xFF);
        if (reg_ptr_ == 0xFF)
        {
            reg_ptr_ = (byte < kibisis::kRegCount) ? byte : 0;
        }
        else
        {
            if (reg_ptr_ < kibisis::kRegCount)
                live_regs_[reg_ptr_++] = byte;
        }
    }

    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS)
    {
        const uint8_t val = (reg_ptr_ < kibisis::kRegCount) ? live_regs_[reg_ptr_++] : 0;
        i2c0->hw->data_cmd = val;
        (void)i2c0->hw->clr_rd_req;
    }

    if (status & I2C_IC_INTR_STAT_R_STOP_DET_BITS)
    {
    if (status & I2C_IC_INTR_STAT_R_STOP_DET_BITS)
    {
        reg_ptr_ = 0xFF;
        (void)i2c0->hw->clr_stop_det;
    }
}

int8_t I2CSlave::getMotorASpeed()     const { return static_cast<int8_t>(live_regs_[kibisis::kRegMotorASpeed]);     }
int8_t I2CSlave::getMotorBSpeed()     const { return static_cast<int8_t>(live_regs_[kibisis::kRegMotorBSpeed]);     }
int8_t I2CSlave::getSpaceMotorSpeed() const { return static_cast<int8_t>(live_regs_[kibisis::kRegSpaceMotorSpeed]); }

void I2CSlave::setStatus(const uint8_t status)
{
    shadow_regs_[kibisis::kRegStatus] = status;
}

void I2CSlave::setEncoderA(const int32_t counts)
{
    memcpy(const_cast<uint8_t*>(&shadow_regs_[kibisis::kRegEncACount0]), &counts, 4);
}

void I2CSlave::setEncoderB(const int32_t counts)
{
    memcpy(const_cast<uint8_t*>(&shadow_regs_[kibisis::kRegEncBCount0]), &counts, 4);
}

bool I2CSlave::getLdrSampleTrigger()
{
    const uint32_t saved = save_and_disable_interrupts();
    const bool triggered = (live_regs_[kibisis::kRegLdrSample] != 0);
    if (triggered)
        live_regs_[kibisis::kRegLdrSample] = 0;
    restore_interrupts(saved);
    return triggered;
}

void I2CSlave::setLdrAmbientA(const uint16_t value)
{
    shadow_regs_[kibisis::kRegLdrAAmbient0]     = static_cast<uint8_t>(value & 0xFF);
    shadow_regs_[kibisis::kRegLdrAAmbient0 + 1] = static_cast<uint8_t>(value >> 8);
}

void I2CSlave::setLdrAmbientB(const uint16_t value)
{
    shadow_regs_[kibisis::kRegLdrBAmbient0]     = static_cast<uint8_t>(value & 0xFF);
    shadow_regs_[kibisis::kRegLdrBAmbient0 + 1] = static_cast<uint8_t>(value >> 8);
}

void I2CSlave::setLdrIlluminatedA(const uint16_t value)
{
    shadow_regs_[kibisis::kRegLdrAIlluminated0]     = static_cast<uint8_t>(value & 0xFF);
    shadow_regs_[kibisis::kRegLdrAIlluminated0 + 1] = static_cast<uint8_t>(value >> 8);
}

void I2CSlave::setLdrIlluminatedB(const uint16_t value)
{
    shadow_regs_[kibisis::kRegLdrBIlluminated0]     = static_cast<uint8_t>(value & 0xFF);
    shadow_regs_[kibisis::kRegLdrBIlluminated0 + 1] = static_cast<uint8_t>(value >> 8);
}
#pragma once

#include <cstddef>
#include <cstdint>

namespace as7343_driver
{
    // I2C
    constexpr uint8_t AS7343_DEFAULT_ADDRESS = 0x39;
    constexpr uint8_t AS7343_DEVICE_ID = 0x81;

    // Bank 1 registers (CFG0 bit 4 = 1)
    constexpr uint8_t REG_AUXID = 0x58;
    constexpr uint8_t REG_REVID = 0x59;
    constexpr uint8_t REG_ID = 0x5A;
    constexpr uint8_t REG_CFG10 = 0x65;
    constexpr uint8_t REG_CFG12 = 0x66;
    constexpr uint8_t REG_GPIO = 0x6B;

    // Bank 0 registers (CFG0 bit 4 = 0, default)
    constexpr uint8_t REG_ENABLE = 0x80;
    constexpr uint8_t REG_ATIME = 0x81;
    constexpr uint8_t REG_WTIME = 0x83;
    constexpr uint8_t REG_SP_TH_L_L = 0x84;
    constexpr uint8_t REG_SP_TH_L_H = 0x85;
    constexpr uint8_t REG_SP_TH_H_L = 0x86;
    constexpr uint8_t REG_SP_TH_H_H = 0x87;
    constexpr uint8_t REG_STATUS2 = 0x90;
    constexpr uint8_t REG_STATUS3 = 0x91;
    constexpr uint8_t REG_STATUS = 0x93;
    constexpr uint8_t REG_ASTATUS = 0x94;
    constexpr uint8_t REG_DATA0_L = 0x95;
    constexpr uint8_t REG_DATA0_H = 0x96;
    constexpr uint8_t REG_STATUS5 = 0xBB;
    constexpr uint8_t REG_STATUS4 = 0xBC;
    constexpr uint8_t REG_CFG0 = 0xBF;
    constexpr uint8_t REG_CFG1 = 0xC6;
    constexpr uint8_t REG_CFG3 = 0xC7;
    constexpr uint8_t REG_CFG8 = 0xC9;
    constexpr uint8_t REG_CFG9 = 0xCA;
    constexpr uint8_t REG_LED = 0xCD;
    constexpr uint8_t REG_PERS = 0xCF;
    constexpr uint8_t REG_ASTEP_L = 0xD4;
    constexpr uint8_t REG_ASTEP_H = 0xD5;
    constexpr uint8_t REG_CFG20 = 0xD6;
    constexpr uint8_t REG_AGC_GAIN_MAX = 0xD7;
    constexpr uint8_t REG_AZ_CONFIG = 0xDE;
    constexpr uint8_t REG_FD_CFG0 = 0xDF;
    constexpr uint8_t REG_FD_TIME1 = 0xE0;
    constexpr uint8_t REG_FD_TIME2 = 0xE2;
    constexpr uint8_t REG_FD_STATUS = 0xE3;
    constexpr uint8_t REG_CFG6 = 0xF5;
    constexpr uint8_t REG_INT_ENAB = 0xF9;
    constexpr uint8_t REG_CONTROL = 0xFA;
    constexpr uint8_t REG_FIFO_MAP = 0xFC;
    constexpr uint8_t REG_FIFO_LVL = 0xFD;
    constexpr uint8_t REG_FDATA_L = 0xFE;
    constexpr uint8_t REG_FDATA_H = 0xFF;

    // ENABLE register bits
    constexpr uint8_t ENABLE_PON = 0x01;
    constexpr uint8_t ENABLE_SP_EN = 0x02;
    constexpr uint8_t ENABLE_WEN = 0x08;
    constexpr uint8_t ENABLE_SMUXEN = 0x10;
    constexpr uint8_t ENABLE_FDEN = 0x40;

    // STATUS2 register bits
    constexpr uint8_t STATUS2_ASAT_ANALOG = 0x01;
    constexpr uint8_t STATUS2_ASAT_DIGITAL = 0x08;
    constexpr uint8_t STATUS2_AVALID = 0x40;

    // STATUS register bits
    constexpr uint8_t STATUS_SINT = 0x01;
    constexpr uint8_t STATUS_AINT = 0x08;

    // STATUS4 register bits
    constexpr uint8_t STATUS4_SP_TRIG = 0x04;
    constexpr uint8_t STATUS4_OVER_TEMP = 0x20;
    constexpr uint8_t STATUS4_FIFO_OV = 0x02;
    constexpr uint8_t STATUS4_SP_BUSY = 0x01;

    // FD_STATUS register bits
    constexpr uint8_t FD_STATUS_100HZ_DET = 0x01;
    constexpr uint8_t FD_STATUS_120HZ_DET = 0x02;
    constexpr uint8_t FD_STATUS_100HZ_VALID = 0x04;
    constexpr uint8_t FD_STATUS_120HZ_VALID = 0x08;
    constexpr uint8_t FD_STATUS_FD_MEAS_VALID = 0x20;
    constexpr uint8_t FD_STATUS_FD_SATURATION = 0x40;

    // CFG0 register bits
    constexpr uint8_t CFG0_REG_BANK = 0x10;
    constexpr uint8_t CFG0_LOW_POWER = 0x20;

    // CFG20 auto-SMUX mode bits (bits 6:5)
    constexpr uint8_t CFG20_SMUX_6CH = 0x00;
    constexpr uint8_t CFG20_SMUX_12CH = 0x40;
    constexpr uint8_t CFG20_SMUX_18CH = 0x60;

    // CONTROL register bits
    constexpr uint8_t CONTROL_SW_RESET = 0x08;
    constexpr uint8_t CONTROL_FIFO_CLR = 0x02;

    // Gain values for CFG1 register (bits 4:0)
    enum class Gain : uint8_t
    {
        GAIN_0_5X = 0x00,
        GAIN_1X = 0x01,
        GAIN_2X = 0x02,
        GAIN_4X = 0x03,
        GAIN_8X = 0x04,
        GAIN_16X = 0x05,
        GAIN_32X = 0x06,
        GAIN_64X = 0x07,
        GAIN_128X = 0x08,
        GAIN_256X = 0x09,
        GAIN_512X = 0x0A,
        GAIN_1024X = 0x0B,
        GAIN_2048X = 0x0C,
    };

    // Convert a gain multiplier to the register enum value
    inline Gain gain_from_multiplier(uint16_t multiplier)
    {
        switch (multiplier)
        {
        case 1:
            return Gain::GAIN_1X;
        case 2:
            return Gain::GAIN_2X;
        case 4:
            return Gain::GAIN_4X;
        case 8:
            return Gain::GAIN_8X;
        case 16:
            return Gain::GAIN_16X;
        case 32:
            return Gain::GAIN_32X;
        case 64:
            return Gain::GAIN_64X;
        case 128:
            return Gain::GAIN_128X;
        case 256:
            return Gain::GAIN_256X;
        case 512:
            return Gain::GAIN_512X;
        case 1024:
            return Gain::GAIN_1024X;
        case 2048:
            return Gain::GAIN_2048X;
        default:
            return Gain::GAIN_256X;
        }
    }

    // Convert gain register value to multiplier
    inline uint16_t gain_to_multiplier(Gain gain)
    {
        switch (gain)
        {
        case Gain::GAIN_0_5X:
            return 1;  // 0.5x reported as 1 for uint16
        case Gain::GAIN_1X:
            return 1;
        case Gain::GAIN_2X:
            return 2;
        case Gain::GAIN_4X:
            return 4;
        case Gain::GAIN_8X:
            return 8;
        case Gain::GAIN_16X:
            return 16;
        case Gain::GAIN_32X:
            return 32;
        case Gain::GAIN_64X:
            return 64;
        case Gain::GAIN_128X:
            return 128;
        case Gain::GAIN_256X:
            return 256;
        case Gain::GAIN_512X:
            return 512;
        case Gain::GAIN_1024X:
            return 1024;
        case Gain::GAIN_2048X:
            return 2048;
        default:
            return 256;
        }
    }

    // 18-channel data register mapping
    // Each measurement cycle reads 6 channels into data registers 0-17
    // Data is organized as: [Cycle1: 0-5] [Cycle2: 6-11] [Cycle3: 12-17]
    //
    // Index  Channel    Wavelength  Cycle
    // 0      FZ         450nm       1
    // 1      FY         555nm       1
    // 2      FXL        600nm       1
    // 3      NIR        855nm       1
    // 4      VIS_1      Clear       1
    // 5      FD_1       Flicker     1
    // 6      F2         425nm       2
    // 7      F3         475nm       2
    // 8      F4         515nm       2
    // 9      F6         640nm       2
    // 10     VIS_2      Clear       2
    // 11     FD_2       Flicker     2
    // 12     F1         405nm       3
    // 13     F7         690nm       3
    // 14     F8         745nm       3
    // 15     F5         550nm       3
    // 16     VIS_3      Clear       3
    // 17     FD_3       Flicker     3

    constexpr size_t AS7343_NUM_DATA_REGISTERS = 18;
    constexpr size_t AS7343_DATA_BYTES = AS7343_NUM_DATA_REGISTERS * 2;  // 36 bytes

    // Data register indices for each spectral channel
    constexpr size_t IDX_FZ = 0;
    constexpr size_t IDX_FY = 1;
    constexpr size_t IDX_FXL = 2;
    constexpr size_t IDX_NIR = 3;
    constexpr size_t IDX_VIS_1 = 4;
    constexpr size_t IDX_FD_1 = 5;
    constexpr size_t IDX_F2 = 6;
    constexpr size_t IDX_F3 = 7;
    constexpr size_t IDX_F4 = 8;
    constexpr size_t IDX_F6 = 9;
    constexpr size_t IDX_VIS_2 = 10;
    constexpr size_t IDX_FD_2 = 11;
    constexpr size_t IDX_F1 = 12;
    constexpr size_t IDX_F7 = 13;
    constexpr size_t IDX_F8 = 14;
    constexpr size_t IDX_F5 = 15;
    constexpr size_t IDX_VIS_3 = 16;
    constexpr size_t IDX_FD_3 = 17;

    // Integration time constant: each step = 2.78 microseconds
    constexpr double ASTEP_PERIOD_US = 2.78;

}  // namespace as7343_driver

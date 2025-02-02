#pragma once

#include <cstdint>

namespace st3215
{
    /**
     * @brief Memory table address definitions for ST3215 servo
     * @details Contains all EPROM and SRAM register addresses and their properties
     */
    namespace memory_table
    {
        /**
         * @brief EPROM read-only register addresses
         */
        struct eprom_readonly_t
        {
            static constexpr uint8_t FIRMWARE_MAIN_VERSION = 0x00;
            static constexpr uint8_t FIRMWARE_SUB_VERSION = 0x01;
            static constexpr uint8_t SERVO_MAIN_VERSION = 0x03;
            static constexpr uint8_t SERVO_SUB_VERSION = 0x04;
        };

        /**
         * @brief EPROM read/write register addresses and limits
         */
        struct eprom_t
        {
            // Address constants
            static constexpr uint8_t ID = 0x05;
            static constexpr uint8_t BAUD_RATE = 0x06;
            static constexpr uint8_t RETURN_DELAY_TIME = 0x07;
            static constexpr uint8_t STATUS_RETURN_LEVEL = 0x08;
            static constexpr uint8_t MIN_POSITION_LIMIT = 0x09;  // 2 bytes
            static constexpr uint8_t MAX_POSITION_LIMIT = 0x0B;  // 2 bytes
            static constexpr uint8_t MAX_TEMPERATURE_LIMIT = 0x0D;
            static constexpr uint8_t MAX_INPUT_VOLTAGE = 0x0E;
            static constexpr uint8_t MIN_INPUT_VOLTAGE = 0x0F;
            static constexpr uint8_t MAX_TORQUE_LIMIT = 0x10;  // 2 bytes
            static constexpr uint8_t CW_DEAD_BAND = 0x1A;
            static constexpr uint8_t CCW_DEAD_BAND = 0x1B;
            static constexpr uint8_t POSITION_OFFSET = 0x1F;  // 2 bytes
            static constexpr uint8_t OPERATING_MODE = 0x21;

            // Range limits
            static constexpr uint8_t ID_MIN = 0;
            static constexpr uint8_t ID_MAX = 253;
            static constexpr uint8_t BAUD_RATE_MIN = 0;
            static constexpr uint8_t BAUD_RATE_MAX = 7;
            static constexpr uint8_t RETURN_DELAY_TIME_MAX = 254;
            static constexpr uint16_t POSITION_MIN = 0;
            static constexpr uint16_t POSITION_MAX = 1023;
            static constexpr uint8_t TEMPERATURE_MAX = 100;
            static constexpr uint8_t VOLTAGE_MAX = 140;
            static constexpr uint8_t VOLTAGE_MIN = 80;
            static constexpr uint16_t TORQUE_MAX = 1000;
            static constexpr uint8_t DEAD_BAND_MAX = 32;
            static constexpr int16_t POSITION_OFFSET_MIN = -2047;
            static constexpr int16_t POSITION_OFFSET_MAX = 2047;

            // Conversion factors
            static constexpr float VOLTAGE_SCALE = 0.1f;  // Voltage units are 0.1V
        };

        /**
         * @brief SRAM read/write register addresses and limits
         */
        struct sram_t
        {
            // Address constants
            static constexpr uint8_t TORQUE_ENABLE = 0x28;
            static constexpr uint8_t ACCELERATION = 0x29;
            static constexpr uint8_t GOAL_POSITION = 0x2A;  // 2 bytes
            static constexpr uint8_t RUNNING_TIME = 0x2C;   // 2 bytes
            static constexpr uint8_t GOAL_SPEED = 0x2E;     // 2 bytes
            static constexpr uint8_t TORQUE_LIMIT = 0x30;   // 2 bytes
            static constexpr uint8_t LOCK = 0x37;

            // Range limits
            static constexpr uint8_t ACCELERATION_MAX = 254;
            static constexpr int16_t RUNNING_TIME_MIN = -32766;
            static constexpr int16_t RUNNING_TIME_MAX = 32766;
            static constexpr int16_t SPEED_MIN = -1000;
            static constexpr int16_t SPEED_MAX = 1000;
        };

        /**
         * @brief SRAM read-only status register addresses
         */
        struct sram_status_t
        {
            static constexpr uint8_t PRESENT_POSITION = 0x38;  // 2 bytes
            static constexpr uint8_t PRESENT_SPEED = 0x3A;     // 2 bytes
            static constexpr uint8_t PRESENT_LOAD = 0x3C;      // 2 bytes
            static constexpr uint8_t PRESENT_VOLTAGE = 0x3E;
            static constexpr uint8_t PRESENT_TEMPERATURE = 0x3F;
            static constexpr uint8_t MOVING_STATUS = 0x42;
            static constexpr uint8_t PRESENT_CURRENT = 0x45;  // 2 bytes
        };

        /**
         * @brief Get baud rate value in bps from EPROM baud rate setting
         * @param baud_setting The baud rate setting (0-7)
         * @return uint32_t The actual baud rate in bps
         * @throws std::out_of_range if baud_setting is invalid
         */
        constexpr uint32_t get_baud_rate(uint8_t baud_setting)
        {
            constexpr uint32_t baud_rates[] = {
                9600,
                19200,
                38400,
                57600,
                115200,
                500000,
                1000000,
                2000000};

            if (baud_setting > eprom_t::BAUD_RATE_MAX)
            {
                throw std::out_of_range("Invalid baud rate setting");
            }

            return baud_rates[baud_setting];
        }

        /**
         * @brief Calculate the size in bytes for a given register
         * @param address The register address
         * @return uint8_t The size in bytes (1 or 2)
         */
        constexpr uint8_t get_register_size(uint8_t address)
        {
            // Check 2-byte registers
            if (address == eprom_t::MIN_POSITION_LIMIT ||
                address == eprom_t::MAX_POSITION_LIMIT ||
                address == eprom_t::MAX_TORQUE_LIMIT ||
                address == eprom_t::POSITION_OFFSET ||
                address == sram_t::GOAL_POSITION ||
                address == sram_t::RUNNING_TIME ||
                address == sram_t::GOAL_SPEED ||
                address == sram_t::TORQUE_LIMIT ||
                address == sram_status_t::PRESENT_POSITION ||
                address == sram_status_t::PRESENT_SPEED ||
                address == sram_status_t::PRESENT_LOAD ||
                address == sram_status_t::PRESENT_CURRENT)
            {
                return 2;
            }
            return 1;
        }
    }  // namespace memory_table
}  // namespace st3215
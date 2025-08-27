#pragma once

#include <cstdint>

namespace perseus
{
    namespace servo
    {
        // ST3215 Servo Memory Map Constants

        // EPROM Register Addresses
        namespace register_addr
        {
            // Read-Only Registers
            static constexpr uint8_t FIRMWARE_MAIN_VERSION = 0x00;
            static constexpr uint8_t FIRMWARE_SUB_VERSION = 0x01;
            static constexpr uint8_t SERVO_MAIN_VERSION = 0x03;
            static constexpr uint8_t SERVO_SUB_VERSION = 0x04;

            // Read/Write Registers
            static constexpr uint8_t ID = 0x05;
            static constexpr uint8_t BAUD_RATE = 0x06;
            static constexpr uint8_t RETURN_DELAY_TIME = 0x07;
            static constexpr uint8_t STATUS_RETURN_LEVEL = 0x08;
            static constexpr uint8_t MIN_POSITION_LIMIT = 0x09;
            static constexpr uint8_t MAX_POSITION_LIMIT = 0x0B;
            static constexpr uint8_t MAX_TEMPERATURE_LIMIT = 0x0D;
            static constexpr uint8_t MAX_INPUT_VOLTAGE = 0x0E;
            static constexpr uint8_t MIN_INPUT_VOLTAGE = 0x0F;
            static constexpr uint8_t MAX_TORQUE_LIMIT = 0x10;
            static constexpr uint8_t CW_DEAD_BAND = 0x1A;
            static constexpr uint8_t CCW_DEAD_BAND = 0x1B;
            static constexpr uint8_t POSITION_OFFSET = 0x1F;
            static constexpr uint8_t OPERATING_MODE = 0x21;

            // SRAM Register Addresses
            static constexpr uint8_t TORQUE_ENABLE = 0x28;
            static constexpr uint8_t ACCELERATION = 0x29;
            static constexpr uint8_t GOAL_POSITION = 0x2A;
            static constexpr uint8_t RUNNING_TIME = 0x2C;
            static constexpr uint8_t GOAL_SPEED = 0x2E;
            static constexpr uint8_t TORQUE_LIMIT = 0x30;
            static constexpr uint8_t LOCK = 0x37;

            // Read-Only Status Registers
            static constexpr uint8_t PRESENT_POSITION = 0x38;
            static constexpr uint8_t PRESENT_SPEED = 0x3A;
            static constexpr uint8_t PRESENT_LOAD = 0x3C;
            static constexpr uint8_t PRESENT_VOLTAGE = 0x3E;
            static constexpr uint8_t PRESENT_TEMPERATURE = 0x3F;
            static constexpr uint8_t MOVING_STATUS = 0x42;
            static constexpr uint8_t PRESENT_CURRENT = 0x45;
        }

        // Instruction Set
        namespace instruction
        {
            static constexpr uint8_t PING = 0x01;
            static constexpr uint8_t READ = 0x02;
            static constexpr uint8_t WRITE = 0x03;
            static constexpr uint8_t REG_WRITE = 0x04;
            static constexpr uint8_t ACTION = 0x05;
            static constexpr uint8_t SYNC_WRITE = 0x83;
        }

        // Protocol Constants
        namespace protocol
        {
            static constexpr uint8_t HEADER1 = 0xFF;
            static constexpr uint8_t HEADER2 = 0xFF;
            static constexpr uint8_t BROADCAST_ID = 0xFE;
        }

        // Limits & Safety Values
        namespace limits
        {
            static constexpr uint16_t MIN_POSITION = 0;
            static constexpr uint16_t MAX_POSITION = 4095;  // 12-bit resolution
            static constexpr int16_t MIN_TORQUE = -1000;
            static constexpr int16_t MAX_TORQUE = 1000;
            static constexpr int16_t TORQUE_SAFETY_THRESHOLD = 800;  // 80% of maximum torque
        }

        // Timing Constants
        namespace timing
        {
            static constexpr auto DEFAULT_TIMEOUT = std::chrono::milliseconds(200);
            static constexpr auto MIN_RESPONSE_TIME = std::chrono::milliseconds(10);
            static constexpr auto PORT_SETTLE_TIME = std::chrono::milliseconds(200);
            static constexpr auto RETRY_DELAY = std::chrono::milliseconds(100);
            static constexpr auto COMMAND_INTERVAL = std::chrono::milliseconds(5);
        }

        // Communication Parameters
        namespace communication
        {
            static constexpr unsigned int DEFAULT_BAUD_RATE = 115200;
            static constexpr int MAX_RETRIES = 3;
            static constexpr uint8_t DEFAULT_ACCELERATION = 30;
        }

        // UI Constants
        namespace ui
        {
            static constexpr int16_t SERVO_REFRESH_DELAY_MS = 25;
            static constexpr int MAX_DISPLAY_TORQUE = 100;  // Scale display to ±100
            static constexpr int TORQUE_BAR_WIDTH = 5;
            static constexpr int POSITION_BAR_LENGTH = 40;
        }

        // Error Bits
        namespace error_bits
        {
            static constexpr uint8_t INPUT_VOLTAGE = 0x01;
            static constexpr uint8_t ANGLE_LIMIT = 0x02;
            static constexpr uint8_t OVERHEATING = 0x04;
            static constexpr uint8_t RANGE = 0x08;
            static constexpr uint8_t CHECKSUM = 0x10;
            static constexpr uint8_t OVERLOAD = 0x20;
            static constexpr uint8_t INSTRUCTION = 0x40;
        }

    }  // namespace servo
}  // namespace perseus
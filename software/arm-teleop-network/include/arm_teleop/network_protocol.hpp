#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace arm_teleop
{
    /**
     * @brief Constants and types for the arm teleoperation network protocol
     */
    namespace protocol
    {
        constexpr uint16_t DEFAULT_PORT = 8787;
        constexpr uint8_t PROTOCOL_VERSION = 1;
        constexpr size_t NUM_SERVOS = 6;

        /**
         * @brief Message types for communication between leader and follower
         */
        enum class message_type_t : uint8_t
        {
            HANDSHAKE = 0x01,        // Initial connection handshake
            SERVO_POSITIONS = 0x02,  // Servo position update
            CALIBRATION = 0x03,      // Calibration data
            STATUS = 0x04,           // Status information
            MIRROR_COMMAND = 0x05,   // Command to enable/disable mirroring
            KEEPALIVE = 0xFF         // Keepalive packet
        };

        /**
         * @brief Data structure for servo position update messages
         */
        struct servo_position_msg_t
        {
            uint8_t header[2] = {0xFF, 0xFF};  // Fixed header
            message_type_t msg_type = message_type_t::SERVO_POSITIONS;
            uint8_t protocol_version = PROTOCOL_VERSION;
            uint16_t positions[NUM_SERVOS] = {0};  // Current positions
            int16_t torques[NUM_SERVOS] = {0};     // Current torque values
            uint8_t checksum = 0;                  // Checksum for data verification
        };

        /**
         * @brief Data structure for calibration data messages
         */
        struct calibration_msg_t
        {
            uint8_t header[2] = {0xFF, 0xFF};  // Fixed header
            message_type_t msg_type = message_type_t::CALIBRATION;
            uint8_t protocol_version = PROTOCOL_VERSION;
            uint16_t min_positions[NUM_SERVOS] = {0};  // Min positions
            uint16_t max_positions[NUM_SERVOS] = {0};  // Max positions
            uint8_t checksum = 0;                      // Checksum for data verification
        };

        /**
         * @brief Data structure for mirroring commands
         */
        struct mirror_command_msg_t
        {
            uint8_t header[2] = {0xFF, 0xFF};  // Fixed header
            message_type_t msg_type = message_type_t::MIRROR_COMMAND;
            uint8_t protocol_version = PROTOCOL_VERSION;
            uint8_t servo_id = 0;       // 1-based servo ID
            uint8_t mirror_enable = 0;  // 0 = disable, 1 = enable
            uint8_t checksum = 0;       // Checksum for data verification
        };

        /**
         * @brief Data structure for status updates
         */
        struct status_msg_t
        {
            uint8_t header[2] = {0xFF, 0xFF};  // Fixed header
            message_type_t msg_type = message_type_t::STATUS;
            uint8_t protocol_version = PROTOCOL_VERSION;
            uint8_t error_flags = 0;     // Bit flags for various error states
            uint8_t mirroring_mask = 0;  // Bit mask for which servos are mirroring
            uint8_t checksum = 0;        // Checksum for data verification
        };

        /**
         * @brief Data structure for handshake message
         */
        struct handshake_msg_t
        {
            uint8_t header[2] = {0xFF, 0xFF};  // Fixed header
            message_type_t msg_type = message_type_t::HANDSHAKE;
            uint8_t protocol_version = PROTOCOL_VERSION;
            uint8_t flags = 0;          // Reserved for future use
            char identifier[16] = {0};  // "PERSEUS_ARM_CTRL"
            uint8_t checksum = 0;       // Checksum for data verification
        };

        /**
         * @brief Data structure for keepalive message
         */
        struct keepalive_msg_t
        {
            uint8_t header[2] = {0xFF, 0xFF};  // Fixed header
            message_type_t msg_type = message_type_t::KEEPALIVE;
            uint8_t protocol_version = PROTOCOL_VERSION;
            uint16_t sequence = 0;  // Sequence number
            uint8_t checksum = 0;   // Checksum for data verification
        };

        /**
         * @brief Calculate checksum for a message
         * @param data Pointer to message data
         * @param length Length of message in bytes (excluding checksum)
         * @return Calculated checksum
         */
        uint8_t calculateChecksum(const uint8_t* data, size_t length);

        /**
         * @brief Serialize a message structure to a byte vector
         * @tparam T Message structure type
         * @param msg Message to serialize
         * @return Vector of bytes representing the message
         */
        template <typename T>
        std::vector<uint8_t> serializeMessage(const T& msg);

        /**
         * @brief Deserialize a byte vector to a message structure
         * @tparam T Message structure type
         * @param data Byte vector containing the message
         * @return Deserialized message structure
         * @throws std::runtime_error if data is invalid or corrupted
         */
        template <typename T>
        T deserializeMessage(const std::vector<uint8_t>& data);

        /**
         * @brief Identifies the message type from a raw data buffer
         * @param data Buffer containing message data
         * @param length Length of the buffer
         * @return Message type if valid, or 0 if invalid
         */
        message_type_t identifyMessageType(const uint8_t* data, size_t length);

    }  // namespace protocol
}  // namespace arm_teleop
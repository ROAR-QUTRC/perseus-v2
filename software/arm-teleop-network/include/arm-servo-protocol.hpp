#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace perseus
{
    // Protocol version
    constexpr uint8_t PROTOCOL_VERSION = 1;

    // Default port for arm teleop communication
    constexpr uint16_t DEFAULT_PORT = 7845;

    // Message types
    enum class MessageType : uint8_t
    {
        SERVO_POSITIONS = 0x01,   // Position data for all servos
        SERVO_MIRRORING = 0x02,   // Mirroring state changes
        CALIBRATION_DATA = 0x03,  // Calibration data exchange
        STATUS_INFO = 0x04,       // Status and error information
        HEARTBEAT = 0x05,         // Connection keepalive message
    };

    // Arm status flags
    enum class ArmStatus : uint8_t
    {
        STATUS_OK = 0x00,  // Changed from OK to STATUS_OK to avoid conflicts
        ERROR_SERVO = 0x01,
        ERROR_TORQUE = 0x02,
        ERROR_COMMUNICATION = 0x04
    };

    // Basic message format that all messages will start with
    struct MessageHeader
    {
        uint8_t protocol_version;
        MessageType type;
        uint16_t payload_length;
    };

    // Message for servo positions (leader → follower)
    struct ServoPositionsMessage
    {
        MessageHeader header;
        struct ServoData
        {
            uint16_t position;  // Current position (0-4095)
            int16_t torque;     // Torque value (-1000 to 1000)
        };
        ServoData servos[6];  // Data for all 6 servos
    };

    // Message for servo mirroring state changes (leader → follower)
    struct ServoMirroringMessage
    {
        MessageHeader header;
        uint8_t servo_id;  // 1-based servo ID
        bool mirroring;    // Whether to enable mirroring
    };

    // Message for calibration data (bidirectional)
    struct CalibrationMessage
    {
        MessageHeader header;
        struct ServoCalibration
        {
            uint16_t min;  // Minimum position (0-4095)
            uint16_t max;  // Maximum position (0-4095)
        };
        ServoCalibration servos[6];
    };

    // Message for status information (follower → leader)
    struct StatusMessage
    {
        MessageHeader header;
        ArmStatus status;
        uint8_t error_servo_id;  // 0 if no specific servo error
        std::string error_message;
    };

    // Heartbeat message for connection keepalive (bidirectional)
    struct HeartbeatMessage
    {
        MessageHeader header;
        uint32_t timestamp;  // Unix timestamp
    };

    // Serialize a message to a byte vector
    template <typename MessageType>
    std::vector<uint8_t> serializeMessage(const MessageType& message);

    // Deserialize a byte vector to a message
    template <typename MessageType>
    bool deserializeMessage(const std::vector<uint8_t>& data, MessageType& message);

}  // namespace perseus
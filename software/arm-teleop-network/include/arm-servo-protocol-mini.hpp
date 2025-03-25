// arm-servo-protocol-mini.hpp
#pragma once
#include <cstdint>
#include <vector>

namespace perseus
{
    constexpr uint8_t PROTOCOL_VERSION = 1;
    constexpr uint16_t DEFAULT_PORT = 7845;

    enum class MessageType : uint8_t
    {
        SERVO_POSITIONS = 0x01,
        HEARTBEAT = 0x05
    };

    struct MessageHeader
    {
        uint8_t protocol_version;
        MessageType type;
        uint16_t payload_length;
    };

    struct ServoPositionsMessage
    {
        MessageHeader header;
        uint16_t positions[6];
    };

    struct HeartbeatMessage
    {
        MessageHeader header;
        uint32_t timestamp;
    };

    // Declarations only
    template <typename T>
    std::vector<uint8_t> serializeMessage(const T& message);

    template <typename T>
    bool deserializeMessage(const std::vector<uint8_t>& data, T& message);
}
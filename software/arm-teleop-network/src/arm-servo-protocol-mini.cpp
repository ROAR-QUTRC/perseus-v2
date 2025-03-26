#include "arm-servo-protocol-mini.hpp"

// Template specializations for ServoPositionsMessage serialization/deserialization
namespace perseus
{
    template <>
    std::vector<uint8_t> serializeMessage(const ServoPositionsMessage& message)
    {
        std::vector<uint8_t> data;
        // Reserve space for header (4 bytes) + payload (12 bytes - 2 bytes per position * 6 servos)
        data.reserve(4 + 12);

        // Add header
        data.push_back(message.header.protocol_version);
        data.push_back(static_cast<uint8_t>(message.header.type));
        data.push_back(message.header.payload_length & 0xFF);
        data.push_back((message.header.payload_length >> 8) & 0xFF);

        // Add position data (6 positions, each 2 bytes)
        for (int i = 0; i < 6; ++i)
        {
            // Position (2 bytes, little-endian)
            data.push_back(message.positions[i] & 0xFF);
            data.push_back((message.positions[i] >> 8) & 0xFF);
        }

        return data;
    }

    template <>
    bool deserializeMessage(const std::vector<uint8_t>& data, ServoPositionsMessage& message)
    {
        if (data.size() < 16)  // 4 byte header + 12 bytes position data
            return false;

        // Deserialize header
        message.header.protocol_version = data[0];
        message.header.type = static_cast<MessageType>(data[1]);
        message.header.payload_length = data[2] | (data[3] << 8);

        // Deserialize position data
        for (int i = 0; i < 6; ++i)
        {
            const size_t offset = 4 + i * 2;  // 4 bytes for header, 2 bytes per position
            message.positions[i] = data[offset] | (data[offset + 1] << 8);
        }

        return true;
    }

    // HeartbeatMessage serialization/deserialization
    template <>
    std::vector<uint8_t> serializeMessage(const HeartbeatMessage& message)
    {
        std::vector<uint8_t> data;
        data.reserve(8);  // Header (4) + timestamp (4)

        // Add header
        data.push_back(message.header.protocol_version);
        data.push_back(static_cast<uint8_t>(message.header.type));
        data.push_back(message.header.payload_length & 0xFF);
        data.push_back((message.header.payload_length >> 8) & 0xFF);

        // Add timestamp (4 bytes, little-endian)
        data.push_back(message.timestamp & 0xFF);
        data.push_back((message.timestamp >> 8) & 0xFF);
        data.push_back((message.timestamp >> 16) & 0xFF);
        data.push_back((message.timestamp >> 24) & 0xFF);

        return data;
    }

    template <>
    bool deserializeMessage(const std::vector<uint8_t>& data, HeartbeatMessage& message)
    {
        if (data.size() < 8)  // 4 byte header + 4 byte timestamp
            return false;

        // Deserialize header
        message.header.protocol_version = data[0];
        message.header.type = static_cast<MessageType>(data[1]);
        message.header.payload_length = data[2] | (data[3] << 8);

        // Deserialize timestamp
        message.timestamp = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24);

        return true;
    }
}
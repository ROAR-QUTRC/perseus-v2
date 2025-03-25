// src/arm-servo-protocol-mini.cpp
#include "arm-servo-protocol-mini.hpp"

namespace perseus
{
    template <>
    std::vector<uint8_t> serializeMessage(const ServoPositionsMessage& message)
    {
        std::vector<uint8_t> data(4 + 12);
        data[0] = message.header.protocol_version;
        data[1] = static_cast<uint8_t>(message.header.type);
        data[2] = message.header.payload_length & 0xFF;
        data[3] = (message.header.payload_length >> 8) & 0xFF;
        for (int i = 0; i < 6; ++i)
        {
            data[4 + i * 2] = message.positions[i] & 0xFF;
            data[5 + i * 2] = (message.positions[i] >> 8) & 0xFF;
        }
        return data;
    }

    template <>
    bool deserializeMessage(const std::vector<uint8_t>& data, ServoPositionsMessage& message)
    {
        if (data.size() < 16)
            return false;
        message.header.protocol_version = data[0];
        message.header.type = static_cast<MessageType>(data[1]);
        message.header.payload_length = data[2] | (data[3] << 8);
        for (int i = 0; i < 6; ++i)
        {
            message.positions[i] = data[4 + i * 2] | (data[5 + i * 2] << 8);
        }
        return true;
    }

    template <>
    std::vector<uint8_t> serializeMessage(const HeartbeatMessage& message)
    {
        std::vector<uint8_t> data(8);
        data[0] = message.header.protocol_version;
        data[1] = static_cast<uint8_t>(message.header.type);
        data[2] = message.header.payload_length & 0xFF;
        data[3] = (message.header.payload_length >> 8) & 0xFF;
        data[4] = message.timestamp & 0xFF;
        data[5] = (message.timestamp >> 8) & 0xFF;
        data[6] = (message.timestamp >> 16) & 0xFF;
        data[7] = (message.timestamp >> 24) & 0xFF;
        return data;
    }

    template <>
    bool deserializeMessage(const std::vector<uint8_t>& data, HeartbeatMessage& message)
    {
        if (data.size() < 8)
            return false;
        message.header.protocol_version = data[0];
        message.header.type = static_cast<MessageType>(data[1]);
        message.header.payload_length = data[2] | (data[3] << 8);
        message.timestamp = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24);
        return true;
    }
}
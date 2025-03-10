#include "arm-servo-protocol.hpp"

namespace perseus
{
    // Template specializations for ServoPositionsMessage
    template <>
    std::vector<uint8_t> serializeMessage(const ServoPositionsMessage& message)
    {
        std::vector<uint8_t> data;
        // Reserve space for header (4 bytes) + payload
        data.reserve(4 + sizeof(ServoPositionsMessage::ServoData) * 6);

        // Add header
        data.push_back(message.header.protocol_version);
        data.push_back(static_cast<uint8_t>(message.header.type));
        data.push_back(message.header.payload_length & 0xFF);
        data.push_back((message.header.payload_length >> 8) & 0xFF);

        // Add servo data
        for (int i = 0; i < 6; ++i)
        {
            // Position (2 bytes, little-endian)
            data.push_back(message.servos[i].position & 0xFF);
            data.push_back((message.servos[i].position >> 8) & 0xFF);

            // Torque (2 bytes, little-endian)
            data.push_back(message.servos[i].torque & 0xFF);
            data.push_back((message.servos[i].torque >> 8) & 0xFF);
        }

        return data;
    }

    template <>
    bool deserializeMessage(const std::vector<uint8_t>& data, ServoPositionsMessage& message)
    {
        if (data.size() < 4 + sizeof(ServoPositionsMessage::ServoData) * 6)
            return false;

        // Deserialize header
        message.header.protocol_version = data[0];
        message.header.type = static_cast<MessageType>(data[1]);
        message.header.payload_length = data[2] | (data[3] << 8);

        // Deserialize servo data
        for (int i = 0; i < 6; ++i)
        {
            const size_t offset = 4 + i * 4;  // 4 bytes per servo (2 for position, 2 for torque)
            message.servos[i].position = data[offset] | (data[offset + 1] << 8);
            message.servos[i].torque = data[offset + 2] | (data[offset + 3] << 8);
        }

        return true;
    }

    // Template specializations for ServoMirroringMessage
    template <>
    std::vector<uint8_t> serializeMessage(const ServoMirroringMessage& message)
    {
        std::vector<uint8_t> data;
        data.reserve(6);  // Header (4) + servo_id (1) + mirroring (1)

        // Add header
        data.push_back(message.header.protocol_version);
        data.push_back(static_cast<uint8_t>(message.header.type));
        data.push_back(message.header.payload_length & 0xFF);
        data.push_back((message.header.payload_length >> 8) & 0xFF);

        // Add payload
        data.push_back(message.servo_id);
        data.push_back(message.mirroring ? 1 : 0);

        return data;
    }

    template <>
    bool deserializeMessage(const std::vector<uint8_t>& data, ServoMirroringMessage& message)
    {
        if (data.size() < 6)
            return false;

        // Deserialize header
        message.header.protocol_version = data[0];
        message.header.type = static_cast<MessageType>(data[1]);
        message.header.payload_length = data[2] | (data[3] << 8);

        // Deserialize payload
        message.servo_id = data[4];
        message.mirroring = data[5] != 0;

        return true;
    }

    // Template specializations for CalibrationMessage
    template <>
    std::vector<uint8_t> serializeMessage(const CalibrationMessage& message)
    {
        std::vector<uint8_t> data;
        data.reserve(4 + sizeof(CalibrationMessage::ServoCalibration) * 6);

        // Add header
        data.push_back(message.header.protocol_version);
        data.push_back(static_cast<uint8_t>(message.header.type));
        data.push_back(message.header.payload_length & 0xFF);
        data.push_back((message.header.payload_length >> 8) & 0xFF);

        // Add servo calibration data
        for (int i = 0; i < 6; ++i)
        {
            // Min position (2 bytes, little-endian)
            data.push_back(message.servos[i].min & 0xFF);
            data.push_back((message.servos[i].min >> 8) & 0xFF);

            // Max position (2 bytes, little-endian)
            data.push_back(message.servos[i].max & 0xFF);
            data.push_back((message.servos[i].max >> 8) & 0xFF);
        }

        return data;
    }

    template <>
    bool deserializeMessage(const std::vector<uint8_t>& data, CalibrationMessage& message)
    {
        if (data.size() < 4 + sizeof(CalibrationMessage::ServoCalibration) * 6)
            return false;

        // Deserialize header
        message.header.protocol_version = data[0];
        message.header.type = static_cast<MessageType>(data[1]);
        message.header.payload_length = data[2] | (data[3] << 8);

        // Deserialize servo calibration data
        for (int i = 0; i < 6; ++i)
        {
            const size_t offset = 4 + i * 4;  // 4 bytes per servo (2 for min, 2 for max)
            message.servos[i].min = data[offset] | (data[offset + 1] << 8);
            message.servos[i].max = data[offset + 2] | (data[offset + 3] << 8);
        }

        return true;
    }

    // Template specializations for StatusMessage
    template <>
    std::vector<uint8_t> serializeMessage(const StatusMessage& message)
    {
        std::vector<uint8_t> data;
        data.reserve(6 + message.error_message.size());

        // Add header
        data.push_back(message.header.protocol_version);
        data.push_back(static_cast<uint8_t>(message.header.type));
        data.push_back(message.header.payload_length & 0xFF);
        data.push_back((message.header.payload_length >> 8) & 0xFF);

        // Add status data
        data.push_back(static_cast<uint8_t>(message.status));
        data.push_back(message.error_servo_id);

        // Add error message as null-terminated string
        data.insert(data.end(), message.error_message.begin(), message.error_message.end());
        data.push_back(0);  // Null terminator

        return data;
    }

    template <>
    bool deserializeMessage(const std::vector<uint8_t>& data, StatusMessage& message)
    {
        if (data.size() < 6)
            return false;

        // Deserialize header
        message.header.protocol_version = data[0];
        message.header.type = static_cast<MessageType>(data[1]);
        message.header.payload_length = data[2] | (data[3] << 8);

        // Deserialize status data
        message.status = static_cast<ArmStatus>(data[4]);
        message.error_servo_id = data[5];

        // Deserialize error message if there is one
        if (data.size() > 6)
        {
            // Find null terminator
            size_t end = 6;
            while (end < data.size() && data[end] != 0)
                ++end;

            // Copy message
            message.error_message.assign(data.begin() + 6, data.begin() + end);
        }
        else
        {
            message.error_message.clear();
        }

        return true;
    }

    // Template specializations for HeartbeatMessage
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
        if (data.size() < 8)
            return false;

        // Deserialize header
        message.header.protocol_version = data[0];
        message.header.type = static_cast<MessageType>(data[1]);
        message.header.payload_length = data[2] | (data[3] << 8);

        // Deserialize timestamp
        message.timestamp = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24);

        return true;
    }
}  // namespace perseus
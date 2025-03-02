#include "arm_teleop/network_protocol.hpp"

#include <cstring>
#include <stdexcept>

namespace arm_teleop
{
    namespace protocol
    {

        uint8_t calculateChecksum(const uint8_t* data, size_t length)
        {
            uint8_t checksum = 0;

            // Skip the header bytes (first two bytes)
            for (size_t i = 2; i < length; ++i)
            {
                checksum += data[i];
            }

            // Return the inverted checksum (one's complement)
            return ~checksum;
        }

        message_type_t identifyMessageType(const uint8_t* data, size_t length)
        {
            // Check for minimum message length and correct header
            if (length < 4 || data[0] != 0xFF || data[1] != 0xFF)
            {
                return static_cast<message_type_t>(0);  // Invalid
            }

            // Message type is the third byte
            message_type_t type = static_cast<message_type_t>(data[2]);

            // Validate protocol version
            if (data[3] != PROTOCOL_VERSION)
            {
                return static_cast<message_type_t>(0);  // Invalid version
            }

            return type;
        }

        // Template specializations for each message type

        template <>
        std::vector<uint8_t> serializeMessage<servo_position_msg_t>(const servo_position_msg_t& msg)
        {
            // Calculate the message size
            const size_t message_size = sizeof(msg);

            // Create a vector to hold the serialized data
            std::vector<uint8_t> data(message_size);

            // Make a copy of the message structure
            servo_position_msg_t msg_copy = msg;

            // Set message type and protocol version
            msg_copy.msg_type = message_type_t::SERVO_POSITIONS;
            msg_copy.protocol_version = PROTOCOL_VERSION;

            // Calculate the checksum (excluding the checksum byte itself)
            msg_copy.checksum = calculateChecksum(reinterpret_cast<const uint8_t*>(&msg_copy), message_size - 1);

            // Copy the message data to the vector
            std::memcpy(data.data(), &msg_copy, message_size);

            return data;
        }

        template <>
        servo_position_msg_t deserializeMessage<servo_position_msg_t>(const std::vector<uint8_t>& data)
        {
            // Check if the data size matches the message structure
            const size_t message_size = sizeof(servo_position_msg_t);
            if (data.size() != message_size)
            {
                throw std::runtime_error("Invalid servo position message size");
            }

            // Create a message structure from the data
            servo_position_msg_t msg;
            std::memcpy(&msg, data.data(), message_size);

            // Verify the header
            if (msg.header[0] != 0xFF || msg.header[1] != 0xFF)
            {
                throw std::runtime_error("Invalid servo position message header");
            }

            // Verify the message type
            if (msg.msg_type != message_type_t::SERVO_POSITIONS)
            {
                throw std::runtime_error("Invalid servo position message type");
            }

            // Verify the protocol version
            if (msg.protocol_version != PROTOCOL_VERSION)
            {
                throw std::runtime_error("Unsupported protocol version");
            }

            // Verify the checksum
            uint8_t calculated_checksum = calculateChecksum(data.data(), message_size - 1);
            if (calculated_checksum != msg.checksum)
            {
                throw std::runtime_error("Invalid servo position message checksum");
            }

            return msg;
        }

        template <>
        std::vector<uint8_t> serializeMessage<calibration_msg_t>(const calibration_msg_t& msg)
        {
            // Calculate the message size
            const size_t message_size = sizeof(msg);

            // Create a vector to hold the serialized data
            std::vector<uint8_t> data(message_size);

            // Make a copy of the message structure
            calibration_msg_t msg_copy = msg;

            // Set message type and protocol version
            msg_copy.msg_type = message_type_t::CALIBRATION;
            msg_copy.protocol_version = PROTOCOL_VERSION;

            // Calculate the checksum (excluding the checksum byte itself)
            msg_copy.checksum = calculateChecksum(reinterpret_cast<const uint8_t*>(&msg_copy), message_size - 1);

            // Copy the message data to the vector
            std::memcpy(data.data(), &msg_copy, message_size);

            return data;
        }

        template <>
        calibration_msg_t deserializeMessage<calibration_msg_t>(const std::vector<uint8_t>& data)
        {
            // Check if the data size matches the message structure
            const size_t message_size = sizeof(calibration_msg_t);
            if (data.size() != message_size)
            {
                throw std::runtime_error("Invalid calibration message size");
            }

            // Create a message structure from the data
            calibration_msg_t msg;
            std::memcpy(&msg, data.data(), message_size);

            // Verify the header
            if (msg.header[0] != 0xFF || msg.header[1] != 0xFF)
            {
                throw std::runtime_error("Invalid calibration message header");
            }

            // Verify the message type
            if (msg.msg_type != message_type_t::CALIBRATION)
            {
                throw std::runtime_error("Invalid calibration message type");
            }

            // Verify the protocol version
            if (msg.protocol_version != PROTOCOL_VERSION)
            {
                throw std::runtime_error("Unsupported protocol version");
            }

            // Verify the checksum
            uint8_t calculated_checksum = calculateChecksum(data.data(), message_size - 1);
            if (calculated_checksum != msg.checksum)
            {
                throw std::runtime_error("Invalid calibration message checksum");
            }

            return msg;
        }

        template <>
        std::vector<uint8_t> serializeMessage<mirror_command_msg_t>(const mirror_command_msg_t& msg)
        {
            // Calculate the message size
            const size_t message_size = sizeof(msg);

            // Create a vector to hold the serialized data
            std::vector<uint8_t> data(message_size);

            // Make a copy of the message structure
            mirror_command_msg_t msg_copy = msg;

            // Set message type and protocol version
            msg_copy.msg_type = message_type_t::MIRROR_COMMAND;
            msg_copy.protocol_version = PROTOCOL_VERSION;

            // Calculate the checksum (excluding the checksum byte itself)
            msg_copy.checksum = calculateChecksum(reinterpret_cast<const uint8_t*>(&msg_copy), message_size - 1);

            // Copy the message data to the vector
            std::memcpy(data.data(), &msg_copy, message_size);

            return data;
        }

        template <>
        mirror_command_msg_t deserializeMessage<mirror_command_msg_t>(const std::vector<uint8_t>& data)
        {
            // Check if the data size matches the message structure
            const size_t message_size = sizeof(mirror_command_msg_t);
            if (data.size() != message_size)
            {
                throw std::runtime_error("Invalid mirror command message size");
            }

            // Create a message structure from the data
            mirror_command_msg_t msg;
            std::memcpy(&msg, data.data(), message_size);

            // Verify the header
            if (msg.header[0] != 0xFF || msg.header[1] != 0xFF)
            {
                throw std::runtime_error("Invalid mirror command message header");
            }

            // Verify the message type
            if (msg.msg_type != message_type_t::MIRROR_COMMAND)
            {
                throw std::runtime_error("Invalid mirror command message type");
            }

            // Verify the protocol version
            if (msg.protocol_version != PROTOCOL_VERSION)
            {
                throw std::runtime_error("Unsupported protocol version");
            }

            // Verify the checksum
            uint8_t calculated_checksum = calculateChecksum(data.data(), message_size - 1);
            if (calculated_checksum != msg.checksum)
            {
                throw std::runtime_error("Invalid mirror command message checksum");
            }

            return msg;
        }

        template <>
        std::vector<uint8_t> serializeMessage<status_msg_t>(const status_msg_t& msg)
        {
            // Calculate the message size
            const size_t message_size = sizeof(msg);

            // Create a vector to hold the serialized data
            std::vector<uint8_t> data(message_size);

            // Make a copy of the message structure
            status_msg_t msg_copy = msg;

            // Set message type and protocol version
            msg_copy.msg_type = message_type_t::STATUS;
            msg_copy.protocol_version = PROTOCOL_VERSION;

            // Calculate the checksum (excluding the checksum byte itself)
            msg_copy.checksum = calculateChecksum(reinterpret_cast<const uint8_t*>(&msg_copy), message_size - 1);

            // Copy the message data to the vector
            std::memcpy(data.data(), &msg_copy, message_size);

            return data;
        }

        template <>
        status_msg_t deserializeMessage<status_msg_t>(const std::vector<uint8_t>& data)
        {
            // Check if the data size matches the message structure
            const size_t message_size = sizeof(status_msg_t);
            if (data.size() != message_size)
            {
                throw std::runtime_error("Invalid status message size");
            }

            // Create a message structure from the data
            status_msg_t msg;
            std::memcpy(&msg, data.data(), message_size);

            // Verify the header
            if (msg.header[0] != 0xFF || msg.header[1] != 0xFF)
            {
                throw std::runtime_error("Invalid status message header");
            }

            // Verify the message type
            if (msg.msg_type != message_type_t::STATUS)
            {
                throw std::runtime_error("Invalid status message type");
            }

            // Verify the protocol version
            if (msg.protocol_version != PROTOCOL_VERSION)
            {
                throw std::runtime_error("Unsupported protocol version");
            }

            // Verify the checksum
            uint8_t calculated_checksum = calculateChecksum(data.data(), message_size - 1);
            if (calculated_checksum != msg.checksum)
            {
                throw std::runtime_error("Invalid status message checksum");
            }

            return msg;
        }

        template <>
        std::vector<uint8_t> serializeMessage<handshake_msg_t>(const handshake_msg_t& msg)
        {
            // Calculate the message size
            const size_t message_size = sizeof(msg);

            // Create a vector to hold the serialized data
            std::vector<uint8_t> data(message_size);

            // Make a copy of the message structure
            handshake_msg_t msg_copy = msg;

            // Set message type and protocol version
            msg_copy.msg_type = message_type_t::HANDSHAKE;
            msg_copy.protocol_version = PROTOCOL_VERSION;

            // Calculate the checksum (excluding the checksum byte itself)
            msg_copy.checksum = calculateChecksum(reinterpret_cast<const uint8_t*>(&msg_copy), message_size - 1);

            // Copy the message data to the vector
            std::memcpy(data.data(), &msg_copy, message_size);

            return data;
        }

        template <>
        handshake_msg_t deserializeMessage<handshake_msg_t>(const std::vector<uint8_t>& data)
        {
            // Check if the data size matches the message structure
            const size_t message_size = sizeof(handshake_msg_t);
            if (data.size() != message_size)
            {
                throw std::runtime_error("Invalid handshake message size");
            }

            // Create a message structure from the data
            handshake_msg_t msg;
            std::memcpy(&msg, data.data(), message_size);

            // Verify the header
            if (msg.header[0] != 0xFF || msg.header[1] != 0xFF)
            {
                throw std::runtime_error("Invalid handshake message header");
            }

            // Verify the message type
            if (msg.msg_type != message_type_t::HANDSHAKE)
            {
                throw std::runtime_error("Invalid handshake message type");
            }

            // Verify the protocol version
            if (msg.protocol_version != PROTOCOL_VERSION)
            {
                throw std::runtime_error("Unsupported protocol version");
            }

            // Verify the checksum
            uint8_t calculated_checksum = calculateChecksum(data.data(), message_size - 1);
            if (calculated_checksum != msg.checksum)
            {
                throw std::runtime_error("Invalid handshake message checksum");
            }

            return msg;
        }

        template <>
        std::vector<uint8_t> serializeMessage<keepalive_msg_t>(const keepalive_msg_t& msg)
        {
            // Calculate the message size
            const size_t message_size = sizeof(msg);

            // Create a vector to hold the serialized data
            std::vector<uint8_t> data(message_size);

            // Make a copy of the message structure
            keepalive_msg_t msg_copy = msg;

            // Set message type and protocol version
            msg_copy.msg_type = message_type_t::KEEPALIVE;
            msg_copy.protocol_version = PROTOCOL_VERSION;

            // Calculate the checksum (excluding the checksum byte itself)
            msg_copy.checksum = calculateChecksum(reinterpret_cast<const uint8_t*>(&msg_copy), message_size - 1);

            // Copy the message data to the vector
            std::memcpy(data.data(), &msg_copy, message_size);

            return data;
        }

        template <>
        keepalive_msg_t deserializeMessage<keepalive_msg_t>(const std::vector<uint8_t>& data)
        {
            // Check if the data size matches the message structure
            const size_t message_size = sizeof(keepalive_msg_t);
            if (data.size() != message_size)
            {
                throw std::runtime_error("Invalid keepalive message size");
            }

            // Create a message structure from the data
            keepalive_msg_t msg;
            std::memcpy(&msg, data.data(), message_size);

            // Verify the header
            if (msg.header[0] != 0xFF || msg.header[1] != 0xFF)
            {
                throw std::runtime_error("Invalid keepalive message header");
            }

            // Verify the message type
            if (msg.msg_type != message_type_t::KEEPALIVE)
            {
                throw std::runtime_error("Invalid keepalive message type");
            }

            // Verify the protocol version
            if (msg.protocol_version != PROTOCOL_VERSION)
            {
                throw std::runtime_error("Unsupported protocol version");
            }

            // Verify the checksum
            uint8_t calculated_checksum = calculateChecksum(data.data(), message_size - 1);
            if (calculated_checksum != msg.checksum)
            {
                throw std::runtime_error("Invalid keepalive message checksum");
            }

            return msg;
        }

    }  // namespace protocol
}  // namespace arm_teleop
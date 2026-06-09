#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <numeric>
#include <optional>
#include <span>
#include <vector>

namespace perseus_lite_hardware::protocol
{
    static constexpr double RADIANS_PER_REVOLUTION = 2.0 * std::numbers::pi;
    static constexpr double SECONDS_PER_MINUTE = 60.0;
    static constexpr double RPM_TO_RAD_S = RADIANS_PER_REVOLUTION / SECONDS_PER_MINUTE;
    static constexpr double RAD_S_TO_RPM = SECONDS_PER_MINUTE / RADIANS_PER_REVOLUTION;
    static constexpr uint16_t ENCODER_TICKS_PER_REVOLUTION = 4096;
    static constexpr double RPM_SCALE_FACTOR = 7.5;
    static constexpr int16_t MAX_VELOCITY_RPM = 1000;
    static constexpr int16_t MIN_VELOCITY_RPM = -1000;
    static constexpr uint16_t SIGN_BIT_MASK = 1 << 15;

    static constexpr uint8_t PACKET_HEADER_BYTE = 0xFF;
    static constexpr size_t PACKET_HEADER_SIZE = 2;
    static constexpr size_t PACKET_ID_INDEX = 2;
    static constexpr size_t PACKET_LENGTH_INDEX = 3;
    static constexpr size_t PACKET_MIN_SIZE = 4;  // header(2) + ID(1) + length(1)

    // Status payload layout (offsets within the payload: error byte + the 8
    // bytes read from PRESENT_POSITION_L (0x38) through PRESENT_TEMPERATURE (0x3F))
    static constexpr size_t STATUS_DATA_SIZE = 8;
    static constexpr size_t ERROR_BYTE_INDEX = 0;
    static constexpr size_t POSITION_LOW_BYTE_INDEX = 1;
    static constexpr size_t POSITION_HIGH_BYTE_INDEX = 2;
    static constexpr size_t VELOCITY_LOW_BYTE_INDEX = 3;
    static constexpr size_t VELOCITY_HIGH_BYTE_INDEX = 4;
    static constexpr size_t VOLTAGE_BYTE_INDEX = 7;
    static constexpr size_t TEMPERATURE_BYTE_INDEX = 8;

    enum class Command : uint8_t
    {
        READ = 0x02,
        WRITE = 0x03
    };

    inline uint8_t calculate_checksum(std::span<const uint8_t> body)
    {
        return ~std::accumulate(body.begin(), body.end(), uint8_t{0});
    }

    inline std::vector<uint8_t> build_packet(uint8_t id, Command cmd, std::span<const uint8_t> data)
    {
        std::vector<uint8_t> packet;
        packet.reserve(data.size() + 6);

        packet.push_back(PACKET_HEADER_BYTE);
        packet.push_back(PACKET_HEADER_BYTE);
        packet.push_back(id);
        packet.push_back(static_cast<uint8_t>(data.size() + 2));
        packet.push_back(static_cast<uint8_t>(cmd));
        packet.insert(packet.end(), data.begin(), data.end());

        const uint8_t checksum = calculate_checksum(
            std::span{packet.data() + PACKET_ID_INDEX, packet.size() - PACKET_ID_INDEX});
        packet.push_back(checksum);

        return packet;
    }

    inline int16_t parse_signed_value(uint16_t raw)
    {
        if (raw & SIGN_BIT_MASK)
        {
            return -static_cast<int16_t>(raw & ~SIGN_BIT_MASK);
        }
        return static_cast<int16_t>(raw);
    }

    inline double ticks_to_radians(int16_t ticks)
    {
        return ticks * (RADIANS_PER_REVOLUTION / ENCODER_TICKS_PER_REVOLUTION);
    }

    inline double raw_velocity_to_rad_s(int16_t raw_velocity)
    {
        const double rpm = raw_velocity * (RPM_SCALE_FACTOR / MAX_VELOCITY_RPM);
        return rpm * RPM_TO_RAD_S;
    }

    inline uint16_t encode_servo_velocity(double rad_s)
    {
        const double rpm = rad_s * RAD_S_TO_RPM;
        double scaled = rpm * (MAX_VELOCITY_RPM / RPM_SCALE_FACTOR);
        double clamped = std::clamp(scaled, static_cast<double>(MIN_VELOCITY_RPM),
                                    static_cast<double>(MAX_VELOCITY_RPM));

        auto servo_speed = static_cast<int16_t>(clamped);
        if (servo_speed < 0)
        {
            servo_speed = -servo_speed;
            servo_speed |= static_cast<int16_t>(SIGN_BIT_MASK);
        }
        return static_cast<uint16_t>(servo_speed);
    }

    inline double apply_motor_direction(uint8_t servo_id, double velocity,
                                        std::span<const uint8_t> inverted_servo_ids)
    {
        if (std::find(inverted_servo_ids.begin(), inverted_servo_ids.end(), servo_id) !=
            inverted_servo_ids.end())
        {
            return -velocity;
        }
        return velocity;
    }

    /**
     * @brief A validated packet located inside a raw serial buffer
     * @details payload spans the bytes between the length byte and the
     *          checksum: for a status response that is the error byte followed
     *          by the requested register data.
     */
    struct ExtractedPacket
    {
        uint8_t id;
        std::span<const uint8_t> payload;
    };

    /**
     * @brief Scans a raw serial buffer for valid servo response packets
     * @details Searches for 0xFF 0xFF headers, validates the declared length
     *          against the buffer bounds and verifies the checksum. Invalid or
     *          truncated candidates are skipped. The returned spans alias the
     *          input buffer and must not outlive it.
     */
    inline std::vector<ExtractedPacket> extract_packets(std::span<const uint8_t> buffer)
    {
        std::vector<ExtractedPacket> packets;
        if (buffer.size() < PACKET_MIN_SIZE)
        {
            return packets;
        }

        for (size_t i = 0; i + PACKET_MIN_SIZE <= buffer.size(); ++i)
        {
            if (buffer[i] != PACKET_HEADER_BYTE || buffer[i + 1] != PACKET_HEADER_BYTE)
            {
                continue;
            }

            const uint8_t id = buffer[i + PACKET_ID_INDEX];
            const uint8_t length = buffer[i + PACKET_LENGTH_INDEX];

            // length counts payload + checksum; a zero length cannot carry a checksum
            if (length == 0 || i + PACKET_MIN_SIZE + length > buffer.size())
            {
                continue;
            }

            const uint8_t checksum = calculate_checksum(
                std::span{buffer.data() + i + PACKET_ID_INDEX,
                          static_cast<size_t>(PACKET_MIN_SIZE + length - 1 - PACKET_ID_INDEX)});
            if (checksum != buffer[i + PACKET_MIN_SIZE + length - 1])
            {
                continue;
            }

            packets.push_back({id,
                               std::span{buffer.data() + i + PACKET_MIN_SIZE,
                                         static_cast<size_t>(length - 1)}});

            // Resume scanning after this packet (loop increment adds 1)
            i += PACKET_MIN_SIZE + length - 1;
        }

        return packets;
    }

    /**
     * @brief Decoded servo status read from PRESENT_POSITION_L (0x38) onwards
     */
    struct ServoStatus
    {
        uint8_t error_flags;
        double position_rad;
        double velocity_rad_s;
        double temperature_c;
    };

    /**
     * @brief Decodes a status payload (error byte + 8 register bytes)
     * @return std::nullopt if the payload is too short to be a status response
     * @details The 8 data bytes cover registers 0x38-0x3F: position(2),
     *          speed(2), load(2), voltage(1), temperature(1).
     */
    inline std::optional<ServoStatus> parse_status_payload(std::span<const uint8_t> payload)
    {
        if (payload.size() < 1 + STATUS_DATA_SIZE)
        {
            return std::nullopt;
        }

        const auto raw_pos = static_cast<uint16_t>(
            payload[POSITION_LOW_BYTE_INDEX] |
            (static_cast<uint16_t>(payload[POSITION_HIGH_BYTE_INDEX]) << 8));
        const auto raw_vel = static_cast<uint16_t>(
            payload[VELOCITY_LOW_BYTE_INDEX] |
            (static_cast<uint16_t>(payload[VELOCITY_HIGH_BYTE_INDEX]) << 8));

        return ServoStatus{
            payload[ERROR_BYTE_INDEX],
            ticks_to_radians(parse_signed_value(raw_pos)),
            raw_velocity_to_rad_s(parse_signed_value(raw_vel)),
            static_cast<double>(payload[TEMPERATURE_BYTE_INDEX]),
        };
    }

}  // namespace perseus_lite_hardware::protocol

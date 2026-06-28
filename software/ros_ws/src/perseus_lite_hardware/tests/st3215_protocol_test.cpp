#include "st3215_protocol.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace protocol = perseus_lite_hardware::protocol;

class ST3215ProtocolTest : public ::testing::Test
{
};

TEST_F(ST3215ProtocolTest, ChecksumSingleByte)
{
    const std::array<uint8_t, 1> data{0x00};
    EXPECT_EQ(protocol::calculate_checksum(data), 0xFF);
}

TEST_F(ST3215ProtocolTest, ChecksumMultipleBytes)
{
    const std::array<uint8_t, 3> data{0x01, 0x04, 0x02};
    uint8_t expected = ~(uint8_t)(0x01 + 0x04 + 0x02);
    EXPECT_EQ(protocol::calculate_checksum(data), expected);
}

TEST_F(ST3215ProtocolTest, ChecksumOverflow)
{
    const std::array<uint8_t, 2> data{0xFF, 0x01};
    uint8_t expected = ~(uint8_t)(0xFF + 0x01);
    EXPECT_EQ(protocol::calculate_checksum(data), expected);
}

TEST_F(ST3215ProtocolTest, BuildPacketHeader)
{
    const std::array<uint8_t, 2> data{0x38, 0x08};
    auto packet = protocol::build_packet(1, protocol::Command::READ, data);

    ASSERT_GE(packet.size(), 7u);
    EXPECT_EQ(packet[0], 0xFF);
    EXPECT_EQ(packet[1], 0xFF);
}

TEST_F(ST3215ProtocolTest, BuildPacketIdAndCommand)
{
    const std::array<uint8_t, 2> data{0x38, 0x08};
    auto packet = protocol::build_packet(3, protocol::Command::READ, data);

    EXPECT_EQ(packet[2], 3);
    EXPECT_EQ(packet[4], 0x02);
}

TEST_F(ST3215ProtocolTest, BuildPacketLength)
{
    const std::array<uint8_t, 2> data{0x38, 0x08};
    auto packet = protocol::build_packet(1, protocol::Command::READ, data);

    // Length = data size (2) + command (1) + checksum (1) = 4
    EXPECT_EQ(packet[3], 4);
}

TEST_F(ST3215ProtocolTest, BuildPacketData)
{
    const std::array<uint8_t, 2> data{0x38, 0x08};
    auto packet = protocol::build_packet(1, protocol::Command::READ, data);

    EXPECT_EQ(packet[5], 0x38);
    EXPECT_EQ(packet[6], 0x08);
}

TEST_F(ST3215ProtocolTest, BuildPacketChecksum)
{
    const std::array<uint8_t, 2> data{0x38, 0x08};
    auto packet = protocol::build_packet(1, protocol::Command::READ, data);

    // Checksum is ~(sum of bytes from ID to end of data)
    // Bytes: ID=1, Length=4, CMD=0x02, Data=0x38, 0x08
    uint8_t expected = ~(uint8_t)(1 + 4 + 0x02 + 0x38 + 0x08);
    EXPECT_EQ(packet.back(), expected);
}

TEST_F(ST3215ProtocolTest, BuildPacketTotalSize)
{
    const std::array<uint8_t, 3> data{0x2E, 0x00, 0x00};
    auto packet = protocol::build_packet(2, protocol::Command::WRITE, data);

    // Header(2) + ID(1) + Length(1) + CMD(1) + Data(3) + Checksum(1) = 9
    EXPECT_EQ(packet.size(), 9u);
}

TEST_F(ST3215ProtocolTest, BuildWritePacketCommand)
{
    const std::array<uint8_t, 2> data{0x21, 0x01};
    auto packet = protocol::build_packet(1, protocol::Command::WRITE, data);

    EXPECT_EQ(packet[4], 0x03);
}

TEST_F(ST3215ProtocolTest, ParseSignedValuePositive)
{
    EXPECT_EQ(protocol::parse_signed_value(100), 100);
    EXPECT_EQ(protocol::parse_signed_value(0), 0);
    EXPECT_EQ(protocol::parse_signed_value(2048), 2048);
}

TEST_F(ST3215ProtocolTest, ParseSignedValueNegative)
{
    // Bit 15 set: 0x8064 = sign bit | 100
    EXPECT_EQ(protocol::parse_signed_value(0x8064), -100);
    // 0x8001 = sign bit | 1
    EXPECT_EQ(protocol::parse_signed_value(0x8001), -1);
}

TEST_F(ST3215ProtocolTest, ParseSignedValueZeroWithSignBit)
{
    EXPECT_EQ(protocol::parse_signed_value(0x8000), 0);
}

TEST_F(ST3215ProtocolTest, TicksToRadiansZero)
{
    EXPECT_DOUBLE_EQ(protocol::ticks_to_radians(0), 0.0);
}

TEST_F(ST3215ProtocolTest, TicksToRadiansFullRevolution)
{
    EXPECT_NEAR(protocol::ticks_to_radians(4096), 2.0 * std::numbers::pi, 1e-10);
}

TEST_F(ST3215ProtocolTest, TicksToRadiansHalfRevolution)
{
    EXPECT_NEAR(protocol::ticks_to_radians(2048), std::numbers::pi, 1e-10);
}

TEST_F(ST3215ProtocolTest, TicksToRadiansNegative)
{
    EXPECT_NEAR(protocol::ticks_to_radians(-2048), -std::numbers::pi, 1e-10);
}

TEST_F(ST3215ProtocolTest, RawVelocityToRadSZero)
{
    EXPECT_DOUBLE_EQ(protocol::raw_velocity_to_rad_s(0), 0.0);
}

TEST_F(ST3215ProtocolTest, RawVelocityToRadSPositive)
{
    double result = protocol::raw_velocity_to_rad_s(1000);
    // 1000 * (7.5 / 1000) = 7.5 RPM -> 7.5 * 2π/60 rad/s
    double expected = 7.5 * 2.0 * std::numbers::pi / 60.0;
    EXPECT_NEAR(result, expected, 1e-10);
}

TEST_F(ST3215ProtocolTest, RawVelocityToRadSNegative)
{
    double result = protocol::raw_velocity_to_rad_s(-500);
    double expected = -3.75 * 2.0 * std::numbers::pi / 60.0;
    EXPECT_NEAR(result, expected, 1e-10);
}

TEST_F(ST3215ProtocolTest, EncodeServoVelocityZero)
{
    EXPECT_EQ(protocol::encode_servo_velocity(0.0), 0u);
}

TEST_F(ST3215ProtocolTest, EncodeServoVelocityPositive)
{
    // 1 rad/s -> RPM -> scaled to servo units
    uint16_t result = protocol::encode_servo_velocity(1.0);
    // 1 rad/s * 60/(2π) ≈ 9.549 RPM * (1000/7.5) ≈ 1273 -> clamped to 1000
    EXPECT_EQ(result, 1000u);
}

TEST_F(ST3215ProtocolTest, EncodeServoVelocitySmallPositive)
{
    // Small velocity that doesn't hit the clamp
    uint16_t result = protocol::encode_servo_velocity(0.1);
    // 0.1 rad/s * 60/(2π) ≈ 0.955 RPM * (1000/7.5) ≈ 127
    EXPECT_GT(result, 0u);
    EXPECT_LT(result, 1000u);
}

TEST_F(ST3215ProtocolTest, EncodeServoVelocityNegative)
{
    uint16_t result = protocol::encode_servo_velocity(-0.1);
    // Should have sign bit set (bit 15)
    EXPECT_TRUE(result & 0x8000);
    // Magnitude should match positive case
    uint16_t magnitude = result & 0x7FFF;
    uint16_t positive_result = protocol::encode_servo_velocity(0.1);
    EXPECT_EQ(magnitude, positive_result);
}

TEST_F(ST3215ProtocolTest, EncodeServoVelocityClampPositive)
{
    // Very high velocity should clamp to MAX_VELOCITY_RPM
    uint16_t result = protocol::encode_servo_velocity(100.0);
    EXPECT_EQ(result, 1000u);
}

TEST_F(ST3215ProtocolTest, EncodeServoVelocityClampNegative)
{
    // Very negative velocity should clamp and have sign bit
    uint16_t result = protocol::encode_servo_velocity(-100.0);
    uint16_t magnitude = result & 0x7FFF;
    EXPECT_EQ(magnitude, 1000u);
    EXPECT_TRUE(result & 0x8000);
}

TEST_F(ST3215ProtocolTest, MotorDirectionRightSide)
{
    // Servo IDs 1 and 4 are right-side, no inversion
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(1, 1.0), 1.0);
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(4, 1.0), 1.0);
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(1, -1.0), -1.0);
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(4, -1.0), -1.0);
}

TEST_F(ST3215ProtocolTest, MotorDirectionLeftSide)
{
    // Servo IDs 2 and 3 are left-side, direction is inverted
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(2, 1.0), -1.0);
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(3, 1.0), -1.0);
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(2, -1.0), 1.0);
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(3, -1.0), 1.0);
}

TEST_F(ST3215ProtocolTest, MotorDirectionZero)
{
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(2, 0.0), 0.0);
    EXPECT_DOUBLE_EQ(protocol::apply_motor_direction(1, 0.0), 0.0);
}

TEST_F(ST3215ProtocolTest, VelocityRoundTrip)
{
    // Encode then decode should produce approximately the same value
    // for velocities within the representable range
    double input = 0.3;
    uint16_t encoded = protocol::encode_servo_velocity(input);

    // Decode: strip sign bit, convert back
    int16_t raw;
    if (encoded & 0x8000)
    {
        raw = -static_cast<int16_t>(encoded & 0x7FFF);
    }
    else
    {
        raw = static_cast<int16_t>(encoded);
    }

    double decoded = protocol::raw_velocity_to_rad_s(raw);

    // Round-trip error comes from integer truncation in encode
    EXPECT_NEAR(decoded, input, 0.02);
}

TEST_F(ST3215ProtocolTest, VelocityRoundTripNegative)
{
    double input = -0.5;
    uint16_t encoded = protocol::encode_servo_velocity(input);

    int16_t raw;
    if (encoded & 0x8000)
    {
        raw = -static_cast<int16_t>(encoded & 0x7FFF);
    }
    else
    {
        raw = static_cast<int16_t>(encoded);
    }

    double decoded = protocol::raw_velocity_to_rad_s(raw);
    EXPECT_NEAR(decoded, input, 0.02);
}

TEST_F(ST3215ProtocolTest, BuildPacketAllServoIds)
{
    const std::array<uint8_t, 2> data{0x38, 0x08};
    for (uint8_t id : {1, 2, 3, 4})
    {
        auto packet = protocol::build_packet(id, protocol::Command::READ, data);
        EXPECT_EQ(packet[2], id) << "Packet ID mismatch for servo " << static_cast<int>(id);
    }
}

TEST_F(ST3215ProtocolTest, ParseSignedValueMaxPositive)
{
    // Maximum positive value (all bits except sign bit)
    EXPECT_EQ(protocol::parse_signed_value(0x7FFF), 0x7FFF);
}

TEST_F(ST3215ProtocolTest, ParseSignedValueMaxNegative)
{
    // Maximum negative value: sign bit | 0x7FFF
    EXPECT_EQ(protocol::parse_signed_value(0xFFFF), -0x7FFF);
}

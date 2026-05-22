// Unit tests for the foundational hi-can serialization primitives.
//
// Every device-specific ParameterGroup builds on these. If the primitives are
// wrong, every downstream serializer is wrong silently.

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

#include "hi_can_parameter.hpp"

using namespace hi_can::parameters;

// ---------- scaled_int32_t ------------------------------------------------

// Helper: build the expected big-endian byte sequence for a given int32 value.
static std::vector<uint8_t> be_bytes(int32_t v)
{
    return {
        static_cast<uint8_t>((v >> 24) & 0xFF),
        static_cast<uint8_t>((v >> 16) & 0xFF),
        static_cast<uint8_t>((v >> 8) & 0xFF),
        static_cast<uint8_t>(v & 0xFF),
    };
}
static std::vector<uint8_t> be_bytes(int16_t v)
{
    return {
        static_cast<uint8_t>((v >> 8) & 0xFF),
        static_cast<uint8_t>(v & 0xFF),
    };
}

TEST(ScaledInt32, SerializesAsBigEndian)
{
    scaled_int32_t<1.0> s;
    s.value = 0x01020304;
    EXPECT_EQ(s.serialize_data(), be_bytes(int32_t{0x01020304}));
}

TEST(ScaledInt32, AppliesScalingOnSerialize)
{
    // scaling factor 1000 means a value of 1.5 → 1500 on the wire
    scaled_int32_t<1000.0> s;
    s.value = 1.5;
    EXPECT_EQ(s.serialize_data(), be_bytes(int32_t{1500}));
}

TEST(ScaledInt32, RoundsHalfAwayFromZero)
{
    // round() in <cmath> is "round half away from zero"
    scaled_int32_t<1.0> s;
    s.value = 0.5;
    EXPECT_EQ(s.serialize_data(), be_bytes(int32_t{1}));
    s.value = -0.5;
    EXPECT_EQ(s.serialize_data(), be_bytes(int32_t{-1}));
}

TEST(ScaledInt32, HandlesNegativeValues)
{
    scaled_int32_t<10.0> s;
    s.value = -3.2;
    // -3.2 * 10 = -32 → big-endian int32
    EXPECT_EQ(s.serialize_data(), be_bytes(int32_t{-32}));
}

TEST(ScaledInt32, DeserializeReversesScaling)
{
    scaled_int32_t<1000.0> s;
    s.deserialize_data(be_bytes(int32_t{1500}));
    EXPECT_DOUBLE_EQ(s.value, 1.5);
}

TEST(ScaledInt32, DeserializeHandlesNegatives)
{
    scaled_int32_t<10.0> s;
    s.deserialize_data(be_bytes(int32_t{-32}));
    EXPECT_DOUBLE_EQ(s.value, -3.2);
}

TEST(ScaledInt32, DeserializeHandlesZero)
{
    scaled_int32_t<1000.0> s;
    s.deserialize_data(be_bytes(int32_t{0}));
    EXPECT_DOUBLE_EQ(s.value, 0.0);
}

TEST(ScaledInt32, RoundTripsScaledValues)
{
    scaled_int32_t<100000.0> a;
    a.value = 0.12345;
    scaled_int32_t<100000.0> b;
    b.deserialize_data(a.serialize_data());
    EXPECT_DOUBLE_EQ(b.value, 0.12345);
}

TEST(ScaledInt32, ThrowsOnUndersizedInput)
{
    scaled_int32_t<1.0> s;
    EXPECT_THROW(s.deserialize_data({0x00, 0x01, 0x02}), std::invalid_argument);
}

TEST(ScaledInt32, ThrowsOnOversizedInput)
{
    scaled_int32_t<1.0> s;
    EXPECT_THROW(s.deserialize_data({0, 0, 0, 0, 0}), std::invalid_argument);
}

// ---------- scaled_int16_t ------------------------------------------------

TEST(ScaledInt16, SerializesAsBigEndian)
{
    scaled_int16_t<1.0> s;
    s.value = 0x0102;
    EXPECT_EQ(s.serialize_data(), be_bytes(int16_t{0x0102}));
}

TEST(ScaledInt16, AppliesScalingOnSerialize)
{
    scaled_int16_t<10.0> s;
    s.value = 12.7;
    EXPECT_EQ(s.serialize_data(), be_bytes(int16_t{127}));
}

TEST(ScaledInt16, HandlesNegativeValues)
{
    scaled_int16_t<10.0> s;
    s.value = -3.2;
    EXPECT_EQ(s.serialize_data(), be_bytes(int16_t{-32}));
}

TEST(ScaledInt16, RoundTripsAcrossSignFlip)
{
    scaled_int16_t<50.0> a;
    a.value = -10.5;
    scaled_int16_t<50.0> b;
    b.deserialize_data(a.serialize_data());
    EXPECT_DOUBLE_EQ(b.value, -10.5);
}

TEST(ScaledInt16, ThrowsOnWrongSize)
{
    scaled_int16_t<1.0> s;
    EXPECT_THROW(s.deserialize_data({0x00}), std::invalid_argument);
    EXPECT_THROW(s.deserialize_data({0x00, 0x01, 0x02}), std::invalid_argument);
}

// ---------- SimpleSerializable<T> -----------------------------------------
//
// SimpleSerializable is a raw memcpy of a packed POD struct in NATIVE byte
// order. This is intentionally distinct from scaled_int{16,32}_t which use
// network byte order. Mixing them up is a common bug source — the tests below
// pin the layout.

namespace
{
#pragma pack(push, 1)
    struct probe_pod_t
    {
        uint8_t a;
        uint16_t b;
        int8_t c;
    };
#pragma pack(pop)
}  // namespace

TEST(SimpleSerializable, RoundTripsPodStruct)
{
    SimpleSerializable<probe_pod_t> in;
    in.a = 0xAB;
    in.b = 0x1234;
    in.c = -7;

    SimpleSerializable<probe_pod_t> out(in.serialize_data());

    EXPECT_EQ(out.a, 0xAB);
    EXPECT_EQ(out.b, 0x1234);
    EXPECT_EQ(out.c, -7);
}

TEST(SimpleSerializable, ProducesPackedSize)
{
    // probe_pod_t: 1 + 2 + 1 = 4 bytes, packed.
    SimpleSerializable<probe_pod_t> s;
    s.a = 0;
    s.b = 0;
    s.c = 0;
    EXPECT_EQ(s.serialize_data().size(), 4u);
}

TEST(SimpleSerializable, UsesNativeByteOrder)
{
    // Pin this to little-endian. Every platform the rover runs on (x86_64,
    // aarch64, armhf) is little-endian; if we ever land on a big-endian host
    // this test will (correctly) fail and force us to revisit the wire format.
    SimpleSerializable<probe_pod_t> s;
    s.a = 0x11;
    s.b = 0x2233;  // little-endian → 0x33, 0x22
    s.c = 0x44;

    const std::vector<uint8_t> expected = {0x11, 0x33, 0x22, 0x44};
    EXPECT_EQ(s.serialize_data(), expected);
}

TEST(SimpleSerializable, ThrowsOnSizeMismatch)
{
    SimpleSerializable<probe_pod_t> s;
    EXPECT_THROW(s.deserialize_data({0x00, 0x01, 0x02}), std::invalid_argument);
    EXPECT_THROW(s.deserialize_data({0, 0, 0, 0, 0}), std::invalid_argument);
}

// ---------- wrapped_value_t<T> + SimpleSerializable ------------------------
//
// Used by every PWM / position / bool parameter in the codebase. This is the
// pattern arm and bucket controllers all follow.

TEST(WrappedValue, RoundTripsUint16)
{
    SimpleSerializable<wrapped_value_t<uint16_t>> in;
    in.value = 0xCAFE;
    SimpleSerializable<wrapped_value_t<uint16_t>> out(in.serialize_data());
    EXPECT_EQ(out.value, 0xCAFE);
}

TEST(WrappedValue, RoundTripsInt16Negative)
{
    SimpleSerializable<wrapped_value_t<int16_t>> in;
    in.value = -12345;
    SimpleSerializable<wrapped_value_t<int16_t>> out(in.serialize_data());
    EXPECT_EQ(out.value, -12345);
}

TEST(WrappedValue, RoundTripsUint8)
{
    SimpleSerializable<wrapped_value_t<uint8_t>> in;
    in.value = 0x7F;
    SimpleSerializable<wrapped_value_t<uint8_t>> out(in.serialize_data());
    EXPECT_EQ(out.value, 0x7F);
}

TEST(WrappedValue, RoundTripsBool)
{
    SimpleSerializable<wrapped_value_t<bool>> in;
    in.value = true;
    auto bytes = in.serialize_data();
    ASSERT_EQ(bytes.size(), 1u);
    EXPECT_EQ(bytes[0], 0x01);

    SimpleSerializable<wrapped_value_t<bool>> out(bytes);
    EXPECT_TRUE(out.value);
}

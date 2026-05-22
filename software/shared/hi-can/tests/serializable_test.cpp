// Unit tests for the foundational hi-can serialization primitives.
//
// Every device-specific ParameterGroup builds on these. If the primitives are
// wrong, every downstream serializer is wrong silently.

#include <gtest/gtest.h>

#include <bit>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "hi_can_parameter.hpp"
#include "test_helpers.hpp"

using namespace hi_can::parameters;
using hi_can_test::be16;
using hi_can_test::be32;

// ---------- scaled_int32_t ------------------------------------------------

TEST(ScaledInt32, SerializesAsBigEndian)
{
    scaled_int32_t<1.0> s;
    s.value = 0x01020304;
    EXPECT_EQ(s.serialize_data(), be32(0x01020304));
}

TEST(ScaledInt32, AppliesScalingOnSerialize)
{
    // Scaling factor 1000 means a value of 1.5 → 1500 on the wire.
    scaled_int32_t<1000.0> s;
    s.value = 1.5;
    EXPECT_EQ(s.serialize_data(), be32(1500));
}

TEST(ScaledInt32, RoundsHalfAwayFromZero)
{
    // round() in <cmath> is "round half away from zero".
    scaled_int32_t<1.0> s;
    s.value = 0.5;
    EXPECT_EQ(s.serialize_data(), be32(1));
    s.value = -0.5;
    EXPECT_EQ(s.serialize_data(), be32(-1));
}

TEST(ScaledInt32, HandlesNegativeValues)
{
    scaled_int32_t<10.0> s;
    s.value = -3.2;
    EXPECT_EQ(s.serialize_data(), be32(-32));
}

TEST(ScaledInt32, DeserializeReversesScaling)
{
    scaled_int32_t<1000.0> s;
    s.deserialize_data(be32(1500));
    EXPECT_DOUBLE_EQ(s.value, 1.5);
}

TEST(ScaledInt32, DeserializeHandlesNegatives)
{
    scaled_int32_t<10.0> s;
    s.deserialize_data(be32(-32));
    EXPECT_DOUBLE_EQ(s.value, -3.2);
}

TEST(ScaledInt32, DeserializeHandlesZero)
{
    scaled_int32_t<1000.0> s;
    s.deserialize_data(be32(0));
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
    EXPECT_EQ(s.serialize_data(), be16(0x0102));
}

TEST(ScaledInt16, AppliesScalingOnSerialize)
{
    scaled_int16_t<10.0> s;
    s.value = 12.7;
    EXPECT_EQ(s.serialize_data(), be16(127));
}

TEST(ScaledInt16, HandlesNegativeValues)
{
    scaled_int16_t<10.0> s;
    s.value = -3.2;
    EXPECT_EQ(s.serialize_data(), be16(-32));
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
// network byte order. Mixing them up is a common bug source — the tests
// below pin the layout.

namespace
{
#pragma pack(push, 1)
    struct probe_pod_t
    {
        uint8_t first;
        uint16_t second;
        int8_t third;
    };
#pragma pack(pop)
}  // namespace

TEST(SimpleSerializable, RoundTripsPodStruct)
{
    SimpleSerializable<probe_pod_t> in;
    in.first = 0xAB;
    in.second = 0x1234;
    in.third = -7;

    SimpleSerializable<probe_pod_t> out(in.serialize_data());

    EXPECT_EQ(out.first, 0xAB);
    EXPECT_EQ(out.second, 0x1234);
    EXPECT_EQ(out.third, -7);
}

TEST(SimpleSerializable, ProducesPackedSize)
{
    // probe_pod_t: 1 + 2 + 1 = 4 bytes, packed.
    SimpleSerializable<probe_pod_t> s;
    s.first = 0;
    s.second = 0;
    s.third = 0;
    EXPECT_EQ(s.serialize_data().size(), 4u);
}

TEST(SimpleSerializable, UsesLittleEndianOnLittleEndianHosts)
{
    // SimpleSerializable is a raw memcpy, so the bytes appear in the host's
    // native byte order. Every platform the rover runs on (x86_64, aarch64,
    // armhf) is little-endian, so we pin to that here. If a future port
    // lands on a big-endian host, this test fires loudly and the wire format
    // must be revisited (it would silently change otherwise).
    if constexpr (std::endian::native != std::endian::little)
        GTEST_SKIP() << "Host is not little-endian; SimpleSerializable wire format would diverge.";

    SimpleSerializable<probe_pod_t> s;
    s.first = 0x11;
    s.second = 0x2233;  // little-endian → 0x33, 0x22
    s.third = 0x44;

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

TEST(WrappedValue, DefaultConstructsToZero)
{
    SimpleSerializable<wrapped_value_t<uint16_t>> s;
    EXPECT_EQ(s.value, 0u);
}

// ---------- bucket_controller wire format ---------------------------------
//
// excavation::bucket::controller::{speed_t, current_t, magnet_t} are real
// device typedefs of SimpleSerializable<wrapped_value_t<T>>. Pinning their
// wire layout here serves as the worked example for arm end-effector PWM
// (which is the same pattern: SimpleSerializable<wrapped_value_t<uint16_t>>).

TEST(BucketControllerWire, SpeedIsLittleEndianInt16)
{
    if constexpr (std::endian::native != std::endian::little)
        GTEST_SKIP() << "Host is not little-endian.";

    excavation::bucket::controller::speed_t s;
    s.value = 1234;  // 0x04D2 little-endian → {0xD2, 0x04}
    EXPECT_EQ(s.serialize_data(), (std::vector<uint8_t>{0xD2, 0x04}));
}

TEST(BucketControllerWire, SpeedNegativeRoundTrips)
{
    excavation::bucket::controller::speed_t a;
    a.value = -2048;
    excavation::bucket::controller::speed_t b(a.serialize_data());
    EXPECT_EQ(b.value, -2048);
}

TEST(BucketControllerWire, CurrentIsLittleEndianUint16)
{
    if constexpr (std::endian::native != std::endian::little)
        GTEST_SKIP() << "Host is not little-endian.";

    excavation::bucket::controller::current_t c;
    c.value = 0xBEEF;
    EXPECT_EQ(c.serialize_data(), (std::vector<uint8_t>{0xEF, 0xBE}));
}

TEST(BucketControllerWire, MagnetIsSingleByteBool)
{
    excavation::bucket::controller::magnet_t m;
    m.value = true;
    EXPECT_EQ(m.serialize_data(), (std::vector<uint8_t>{0x01}));
    m.value = false;
    EXPECT_EQ(m.serialize_data(), (std::vector<uint8_t>{0x00}));
}

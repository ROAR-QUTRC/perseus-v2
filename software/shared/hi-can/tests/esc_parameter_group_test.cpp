// Legacy drive ESC parameter group tests.
//
// The startup-transmissions case is the regression test that would have
// caught PR #436 deleting the LIMITS-request behaviour from
// EscParameterGroup::get_startup_transmissions(). Keep it.

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>

#include "hi_can.hpp"
#include "hi_can_address.hpp"
#include "hi_can_packet.hpp"
#include "hi_can_parameter.hpp"
#include "mock_can_interface.hpp"

using namespace hi_can;
using namespace hi_can::parameters::legacy::drive::motors;
// Bring the addressing-side enums (esc::parameter, mcb::groups, device)
// into scope for test bodies. The same namespace tree exists under
// `parameters::legacy::drive::motors` for the group/struct types.
using namespace hi_can::addressing::legacy::drive::motors;
using hi_can_test::FifoCanInterface;

namespace
{
    // Build the device address for FRONT_LEFT_MOTOR's ESC. The legacy
    // addressing layout is system|subsystem|device|group|parameter; for the
    // ESC parameter group, the group is `mcb::groups::ESC` and parameters
    // are SPEED / LIMITS / STATUS / POSITION.
    addressing::legacy::address_t make_device_address()
    {
        using namespace addressing::legacy;
        return address_t(
            drive::SYSTEM_ID,
            drive::motors::SUBSYSTEM_ID,
            static_cast<uint8_t>(drive::motors::device::FRONT_LEFT_MOTOR));
    }

    addressing::flagged_address_t parameter_address(
        const addressing::legacy::address_t& device_address,
        addressing::legacy::drive::motors::esc::parameter param,
        bool is_rtr = false)
    {
        using namespace addressing::legacy;
        return addressing::flagged_address_t{
            address_t(device_address,
                      static_cast<uint8_t>(drive::motors::mcb::groups::ESC),
                      static_cast<uint8_t>(param)),
            is_rtr,
        };
    }
}  // namespace

class EscParameterGroupTest : public ::testing::Test
{
protected:
    addressing::legacy::address_t device_address = make_device_address();
    FifoCanInterface bus;
    std::unique_ptr<PacketManager> packet_manager;
    std::unique_ptr<EscParameterGroup> group;

    void SetUp() override
    {
        packet_manager = std::make_unique<PacketManager>(bus);
        group = std::make_unique<EscParameterGroup>(device_address);
        packet_manager->add_group(*group);
        decltype(bus.transmitted){}.swap(bus.transmitted);
    }
};

// Regression test for PR #436. EscParameterGroup::get_startup_transmissions
// must emit exactly one RTR packet at the LIMITS address. PR #436 commented
// out that body when fighting a namespace clash, silently disabling the
// limits-request on every drive ESC at boot. This test fires loudly if any
// future change does the same.
TEST_F(EscParameterGroupTest, StartupTransmissionsRequestsLimits)
{
    const auto packets = group->get_startup_transmissions();
    ASSERT_EQ(packets.size(), 1u) << "expected one RTR packet for LIMITS";

    const auto expected_address = parameter_address(
        device_address, esc::parameter::LIMITS, /*is_rtr=*/true);
    EXPECT_EQ(packets.front().get_address(), expected_address);
    EXPECT_TRUE(packets.front().get_is_rtr());
    EXPECT_EQ(packets.front().get_data_len(), 0u);
}

// Speed frame is a packed POD: enabled (bool, 1 byte), direction (int8),
// speed (int16 LE). Native byte order — pin little-endian.
TEST_F(EscParameterGroupTest, DeserializesSpeed)
{
    if constexpr (std::endian::native != std::endian::little)
        GTEST_SKIP() << "Host is not little-endian.";

    // Bytes: enabled=true, direction=FORWARD(1), speed=0x0123 LE → {0x23, 0x01}
    const std::vector<uint8_t> payload = {0x01, 0x01, 0x23, 0x01};
    bus.queue_receive(Packet{
        parameter_address(device_address, esc::parameter::SPEED),
        payload});
    packet_manager->handle_receive();

    auto& s = group->get_speed();
    EXPECT_TRUE(s.enabled);
    EXPECT_EQ(static_cast<int8_t>(s.direction),
              static_cast<int8_t>(motor_direction::FORWARD));
    EXPECT_EQ(s.speed, 0x0123);
}

// Status frame: ready (bool, 1 byte), real_speed (int16 LE), real_current
// (int16 LE).
TEST_F(EscParameterGroupTest, DeserializesStatus)
{
    if constexpr (std::endian::native != std::endian::little)
        GTEST_SKIP() << "Host is not little-endian.";

    // ready=true, real_speed=-100 (0xFF9C LE → {0x9C, 0xFF}), real_current=42 (LE → {0x2A, 0x00})
    const std::vector<uint8_t> payload = {0x01, 0x9C, 0xFF, 0x2A, 0x00};
    bus.queue_receive(Packet{
        parameter_address(device_address, esc::parameter::STATUS),
        payload});
    packet_manager->handle_receive();

    auto& s = group->get_status();
    EXPECT_TRUE(s.ready);
    EXPECT_EQ(s.real_speed, -100);
    EXPECT_EQ(s.real_current, 42);
}

// A frame for the same address but a different *device* must be ignored.
TEST_F(EscParameterGroupTest, IgnoresFramesForOtherDevice)
{
    using namespace addressing::legacy;
    const address_t other_device(
        drive::SYSTEM_ID,
        drive::motors::SUBSYSTEM_ID,
        static_cast<uint8_t>(drive::motors::device::FRONT_RIGHT_MOTOR));
    const auto other_address = parameter_address(
        other_device, esc::parameter::SPEED);

    bus.queue_receive(Packet{other_address, std::vector<uint8_t>{0x01, 0x01, 0xFF, 0x7F}});
    packet_manager->handle_receive();

    auto& s = group->get_speed();
    EXPECT_FALSE(s.enabled);
    EXPECT_EQ(s.speed, 0);
}

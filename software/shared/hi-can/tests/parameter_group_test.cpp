// End-to-end test for a real device's ParameterGroup, using the VESC drive ESC
// as the worked example. This is the template every device test should follow:
// queue golden-byte CAN frames on a mock interface, run PacketManager::handle,
// and assert the group's getters decoded as expected.
//
// Golden bytes here are derived directly from the wire-format scaling factors
// documented in hi_can_parameter.cpp (rpm: int32 BE / 1, current: int16 BE /
// 10, etc.), NOT by round-tripping through the code under test. That would
// hide bugs in the serializer.

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <queue>
#include <vector>

#include "hi_can.hpp"
#include "hi_can_address.hpp"
#include "hi_can_packet.hpp"
#include "hi_can_parameter.hpp"

using namespace hi_can;
using namespace hi_can::parameters;
using namespace hi_can::parameters::drive::vesc;

namespace
{
    // Minimal FilteredCanInterface backed by FIFO queues. Replaces the
    // platform CAN driver in tests so we can inject and inspect frames.
    // Mirrors SoftwareFilteredCanInterface::receive: applies the registered
    // filters and dispatches to the receive callback that PacketManager
    // installs at construction.
    class FifoCanInterface : public FilteredCanInterface
    {
    public:
        void transmit(const Packet& packet) override
        {
            transmitted.push(packet);
        }
        std::optional<Packet> receive(bool blocking = false) override
        {
            (void)blocking;
            if (to_receive.empty())
                return std::nullopt;
            Packet p = to_receive.front();
            to_receive.pop();

            if (!address_matches_filters(p.get_address()))
                return std::nullopt;

            if (_receive_callback)
                _receive_callback(p);
            return p;
        }

        void queue_receive(const Packet& p) { to_receive.push(p); }

        std::queue<Packet> to_receive;
        std::queue<Packet> transmitted;
    };

    // Pack an int32 in network byte order into a byte vector. Defined locally
    // so we don't depend on the implementation under test for our golden
    // bytes.
    std::vector<uint8_t> be32(int32_t v)
    {
        return {
            static_cast<uint8_t>((v >> 24) & 0xFF),
            static_cast<uint8_t>((v >> 16) & 0xFF),
            static_cast<uint8_t>((v >> 8) & 0xFF),
            static_cast<uint8_t>(v & 0xFF),
        };
    }
    std::vector<uint8_t> be16(int16_t v)
    {
        return {
            static_cast<uint8_t>((v >> 8) & 0xFF),
            static_cast<uint8_t>(v & 0xFF),
        };
    }
    std::vector<uint8_t> concat(std::initializer_list<std::vector<uint8_t>> parts)
    {
        std::vector<uint8_t> out;
        for (const auto& p : parts)
            out.insert(out.end(), p.begin(), p.end());
        return out;
    }
}  // namespace

namespace
{
    constexpr uint8_t kVescId = 0x01;

    // Build the flagged address for a given VESC command for our test VESC ID.
    addressing::flagged_address_t vesc_address(addressing::drive::vesc::command_id cmd)
    {
        using namespace addressing::drive::vesc;
        return static_cast<addressing::flagged_address_t>(address_t(kVescId, cmd));
    }
}  // namespace

class VescParameterGroupTest : public ::testing::Test
{
protected:
    FifoCanInterface bus;
    std::unique_ptr<PacketManager> packet_manager;
    std::unique_ptr<VescParameterGroup> group;

    void SetUp() override
    {
        packet_manager = std::make_unique<PacketManager>(bus);
        group = std::make_unique<VescParameterGroup>(kVescId);
        packet_manager->add_group(*group);
    }

    // Inject a frame at the given VESC address and run one handle cycle.
    void inject(addressing::drive::vesc::command_id cmd,
                const std::vector<uint8_t>& payload)
    {
        bus.queue_receive(Packet{vesc_address(cmd), payload});
        packet_manager->handle();
    }
};

// Status 1: rpm (int32 BE, scale 1), current (int16 BE, scale 10),
// duty_cycle (int16 BE, scale 1000). 8 bytes total.
TEST_F(VescParameterGroupTest, DeserializesStatus1)
{
    // rpm = 2500, current = 4.2 A, duty_cycle = 0.157 (15.7%)
    const auto payload = concat({
        be32(2500),  // rpm
        be16(42),    // current * 10
        be16(157),   // duty_cycle * 1000
    });

    inject(addressing::drive::vesc::command_id::STATUS_1, payload);

    auto& s = group->get_status1();
    EXPECT_DOUBLE_EQ(s.rpm, 2500.0);
    EXPECT_DOUBLE_EQ(s.current, 4.2);
    EXPECT_DOUBLE_EQ(s.duty_cycle, 0.157);
}

// Status 2: amp-hours and amp-hours-charged, both int32 BE scaled by 10000.
TEST_F(VescParameterGroupTest, DeserializesStatus2)
{
    const auto payload = concat({
        be32(15000),  // ah * 10000 → 1.5 Ah
        be32(2500),   // ah_charge * 10000 → 0.25 Ah
    });

    inject(addressing::drive::vesc::command_id::STATUS_2, payload);

    auto& s = group->get_status2();
    EXPECT_DOUBLE_EQ(s.ah, 1.5);
    EXPECT_DOUBLE_EQ(s.ah_charge, 0.25);
}

// Status 4: temp_fet/temp_motor/current_in (int16 BE / 10), pid_pos (int16 BE / 50).
// Only positive values exercised here — status_4's int16 fields drop the
// `static_cast<int16_t>` after `ntohs` that status_1 has, so negative wire
// values currently decode as huge positives. See
// DISABLED_DeserializesStatus4NegativeCurrentIn below.
TEST_F(VescParameterGroupTest, DeserializesStatus4)
{
    const auto payload = concat({
        be16(425),   // temp_fet * 10 → 42.5 C
        be16(380),   // temp_motor * 10 → 38.0 C
        be16(127),   // current_in * 10 → 12.7 A
        be16(9000),  // pid_pos * 50 → 180.0 deg
    });

    inject(addressing::drive::vesc::command_id::STATUS_4, payload);

    auto& s = group->get_status4();
    EXPECT_DOUBLE_EQ(s.temp_fet, 42.5);
    EXPECT_DOUBLE_EQ(s.temp_motor, 38.0);
    EXPECT_DOUBLE_EQ(s.current_in, 12.7);
    EXPECT_DOUBLE_EQ(s.pid_pos, 180.0);
}

// status_4_t::deserialize_data computes `ntohs(raw_data.current_in) / 10.0`
// without casting through int16_t. Because `ntohs` returns `uint16_t`, the
// sign bit is lost — a wire value of -127 (0xFF81) decodes as 65409, then
// /10 = 6540.9. status_1 handles this correctly via
// `static_cast<int16_t>(ntohs(...))`; the same fix is needed in
// status_4_t/status_5_t/status_6_t (every int16 field that can be negative).
//
// Status1 also has a deserialize for negative current which DOES work — see
// DeserializesStatus1NegativeCurrent below for the working case.
//
// When the cast is added, remove the DISABLED_ prefix.
TEST_F(VescParameterGroupTest, DISABLED_DeserializesStatus4NegativeCurrentIn)
{
    const auto payload = concat({
        be16(0),
        be16(0),
        be16(-127),  // current_in * 10 → -12.7 A
        be16(0),
    });
    inject(addressing::drive::vesc::command_id::STATUS_4, payload);
    EXPECT_DOUBLE_EQ(group->get_status4().current_in, -12.7);
}

// status_1's current field correctly handles negative values because its
// deserialize uses `static_cast<int16_t>(ntohs(...))`. Pinning this so the
// fix to status_4 doesn't accidentally regress status_1.
TEST_F(VescParameterGroupTest, DeserializesStatus1NegativeCurrent)
{
    const auto payload = concat({
        be32(0),
        be16(-42),  // current * 10 → -4.2 A
        be16(0),
    });
    inject(addressing::drive::vesc::command_id::STATUS_1, payload);
    EXPECT_DOUBLE_EQ(group->get_status1().current, -4.2);
}

// Status 5: tachometer (int32 BE / 6), volts_in (int16 BE / 10), 2 bytes pad.
TEST_F(VescParameterGroupTest, DeserializesStatus5)
{
    const auto payload = concat({
        be32(60000),  // tachometer / 6 → 10000.0
        be16(480),    // volts_in / 10 → 48.0 V
        {0x00, 0x00},
    });

    inject(addressing::drive::vesc::command_id::STATUS_5, payload);

    auto& s = group->get_status5();
    EXPECT_DOUBLE_EQ(s.tachometer, 10000.0);
    EXPECT_DOUBLE_EQ(s.volts_in, 48.0);
}

// A frame addressed to a different VESC ID must not mutate this group.
TEST_F(VescParameterGroupTest, IgnoresFramesForOtherVescId)
{
    using namespace addressing::drive::vesc;
    const auto other_address = static_cast<addressing::flagged_address_t>(
        address_t(static_cast<uint8_t>(kVescId + 1), command_id::STATUS_1));

    bus.queue_receive(Packet{other_address, concat({be32(9999), be16(999), be16(999)})});
    packet_manager->handle();

    auto& s = group->get_status1();
    EXPECT_DOUBLE_EQ(s.rpm, 0.0);
    EXPECT_DOUBLE_EQ(s.current, 0.0);
    EXPECT_DOUBLE_EQ(s.duty_cycle, 0.0);
}

// Setting a SET_RPM value drives a transmission with the correct big-endian
// scaled int32. set_rpm_t uses scaled_int32_t<1.0> so 2500 rpm → BE int32 2500.
TEST_F(VescParameterGroupTest, SetRpmTransmitsScaledBigEndian)
{
    using namespace addressing::drive::vesc;

    group->get_set_rpm().value = 2500.0;
    // Force transmit regardless of interval.
    packet_manager->handle(/*should_block=*/false, /*should_force_transmission=*/true);

    ASSERT_FALSE(bus.transmitted.empty()) << "expected at least one transmission";
    const Packet& tx = bus.transmitted.front();
    EXPECT_EQ(tx.get_address(),
              static_cast<addressing::flagged_address_t>(address_t(kVescId, command_id::SET_RPM)));
    EXPECT_EQ(tx.get_data(), be32(2500));
}

// Round-trip is currently broken: status_*_t::serialize_data() in
// hi_can_parameter.cpp applies htons/htonl BEFORE scaling
// (e.g. `raw_data.current = htons(current) * 10.0;`) so serialize → deserialize
// does NOT return the original value for any status struct except status_1's
// rpm field (which has no fractional scaling).
//
// When that serialize bug is fixed (multiply, then htons; see TODO in
// status_5_t::serialize_data) remove the DISABLED_ prefix from this test.
TEST_F(VescParameterGroupTest, DISABLED_Status1RoundTripsThroughSerialize)
{
    status_1_t in;
    in.rpm = 2500;
    in.current = 4.2;
    in.duty_cycle = 0.157;

    status_1_t out;
    out.deserialize_data(in.serialize_data());

    EXPECT_DOUBLE_EQ(out.rpm, 2500.0);
    EXPECT_DOUBLE_EQ(out.current, 4.2);
    EXPECT_DOUBLE_EQ(out.duty_cycle, 0.157);
}

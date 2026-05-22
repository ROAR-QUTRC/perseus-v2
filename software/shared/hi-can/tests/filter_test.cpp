// Tests for the FilteredCanInterface / PacketManager filter-matching path.
//
// Most existing device parameter groups use exact-address filters, but the
// arm code introduced in PR #436 uses masked filters (e.g. `mask =
// DEVICE_MASK`) so a single callback handles every parameter on a given
// device. Pinning the mask semantics here means downstream tests can rely
// on the dispatch behaviour without re-deriving it.

#include <gtest/gtest.h>

#include <cstdint>

#include "hi_can.hpp"
#include "hi_can_address.hpp"
#include "hi_can_packet.hpp"
#include "mock_can_interface.hpp"

using namespace hi_can;
using namespace hi_can::addressing;
using hi_can_test::FifoCanInterface;

namespace
{
    // Helper: build a flagged extended CAN address from a raw value.
    constexpr flagged_address_t flagged(raw_address_t addr)
    {
        return flagged_address_t{addr};
    }
}  // namespace

// matches() with the default MASK_ALL mask requires every bit to match.
TEST(Filter, ExactMatchRejectsDifferentAddress)
{
    const filter_t f{.address = flagged(0x12345678)};
    EXPECT_TRUE(f.matches(flagged(0x12345678)));
    EXPECT_FALSE(f.matches(flagged(0x12345679)));
    EXPECT_FALSE(f.matches(flagged(0x12340678)));
}

// With DEVICE_MASK (system|subsystem|device bits only), all parameter
// groups and parameters of a given device match the same filter.
TEST(Filter, DeviceMaskMatchesAllGroupsAndParameters)
{
    // Pick an arbitrary device: system=3, subsystem=1, device=5.
    const standard_address_t device_address{
        /*system=*/3, /*subsystem=*/1, /*device=*/5};
    const filter_t f{
        .address = flagged_address_t(static_cast<raw_address_t>(device_address)),
        .mask = DEVICE_MASK,
    };

    // Any group/parameter on that device must match.
    const standard_address_t same_device_a{3, 1, 5, /*group=*/0x00, /*parameter=*/0x00};
    const standard_address_t same_device_b{3, 1, 5, /*group=*/0xAB, /*parameter=*/0xCD};
    EXPECT_TRUE(f.matches(flagged_address_t(static_cast<raw_address_t>(same_device_a))));
    EXPECT_TRUE(f.matches(flagged_address_t(static_cast<raw_address_t>(same_device_b))));

    // A different device (device=6) must NOT match.
    const standard_address_t other_device{3, 1, 6, 0x00, 0x00};
    EXPECT_FALSE(f.matches(flagged_address_t(static_cast<raw_address_t>(other_device))));

    // Same device but different subsystem also must not match.
    const standard_address_t other_subsystem{3, 2, 5, 0x00, 0x00};
    EXPECT_FALSE(f.matches(flagged_address_t(static_cast<raw_address_t>(other_subsystem))));
}

// PacketManager + FilteredCanInterface end-to-end: callbacks registered
// with a masked filter fire for every matching address.
TEST(Filter, PacketManagerDispatchesMaskedFilter)
{
    FifoCanInterface bus;
    PacketManager packet_manager(bus);

    const standard_address_t device_address{3, 1, 5};
    const filter_t device_filter{
        .address = flagged_address_t(static_cast<raw_address_t>(device_address)),
        .mask = DEVICE_MASK,
    };

    int call_count = 0;
    Packet last_packet;
    packet_manager.set_callback(
        device_filter,
        PacketManager::callback_config_t{
            .data_callback = [&](const Packet& p)
            {
                ++call_count;
                last_packet = p;
            },
        });

    // Inject three frames at three different (group, parameter) pairs on
    // the matching device.
    for (uint8_t param : {uint8_t{0x00}, uint8_t{0x11}, uint8_t{0xFF}})
    {
        const standard_address_t addr{3, 1, 5, /*group=*/0x42, param};
        bus.queue_receive(Packet{
            flagged_address_t(static_cast<raw_address_t>(addr)),
            std::vector<uint8_t>{param},
        });
    }

    // Inject a frame for a non-matching device — must be filtered out.
    const standard_address_t other_addr{3, 1, 6, 0x00, 0x00};
    bus.queue_receive(Packet{
        flagged_address_t(static_cast<raw_address_t>(other_addr)),
        std::vector<uint8_t>{0xDE},
    });

    packet_manager.handle_receive();

    EXPECT_EQ(call_count, 3) << "masked filter must fire once per matching frame";
    // Last frame seen should be the param=0xFF one.
    ASSERT_EQ(last_packet.get_data_len(), 1u);
    EXPECT_EQ(last_packet.get_data().front(), 0xFFu);
}

// When two filters cover the same address with different mask specificity,
// PacketManager picks the more specific one (filter_t's operator<=> sorts
// greater-mask filters first inside the std::set).
TEST(Filter, MoreSpecificFilterWinsOnOverlap)
{
    FifoCanInterface bus;
    PacketManager packet_manager(bus);

    const standard_address_t device_address{3, 1, 5};
    const standard_address_t specific_address{3, 1, 5, /*group=*/0x42, /*parameter=*/0xAB};

    int generic_calls = 0;
    int specific_calls = 0;

    packet_manager.set_callback(
        filter_t{
            .address = flagged_address_t(static_cast<raw_address_t>(device_address)),
            .mask = DEVICE_MASK,
        },
        PacketManager::callback_config_t{
            .data_callback = [&](const Packet&)
            { ++generic_calls; },
        });
    packet_manager.set_callback(
        filter_t{
            .address = flagged_address_t(static_cast<raw_address_t>(specific_address)),
            // MASK_ALL is the default — explicit here for documentation.
            .mask = MASK_ALL,
        },
        PacketManager::callback_config_t{
            .data_callback = [&](const Packet&)
            { ++specific_calls; },
        });

    bus.queue_receive(Packet{
        flagged_address_t(static_cast<raw_address_t>(specific_address)),
        std::vector<uint8_t>{},
    });
    packet_manager.handle_receive();

    EXPECT_EQ(specific_calls, 1) << "exact filter should win over masked one";
    EXPECT_EQ(generic_calls, 0);
}

// Mock FilteredCanInterface backed by FIFO queues. Used by every
// parameter-group / filter test to inject and inspect CAN frames without
// touching a real platform CAN driver.
//
// Mirrors SoftwareFilteredCanInterface::receive: applies the registered
// filters, then dispatches to the receive callback PacketManager installs
// at construction. Without that dispatch the PacketManager's
// `_handle_received_packet` would never be called and tests would silently
// observe defaults.

#pragma once

#include <optional>
#include <queue>

#include "hi_can.hpp"
#include "hi_can_packet.hpp"

namespace hi_can_test
{
    class FifoCanInterface : public hi_can::FilteredCanInterface
    {
    public:
        void transmit(const hi_can::Packet& packet) override
        {
            transmitted.push(packet);
        }

        std::optional<hi_can::Packet> receive(bool blocking = false) override
        {
            (void)blocking;
            if (to_receive.empty())
                return std::nullopt;
            hi_can::Packet p = to_receive.front();
            to_receive.pop();

            if (!address_matches_filters(p.get_address()))
                return std::nullopt;

            if (_receive_callback)
                _receive_callback(p);
            return p;
        }

        void queue_receive(const hi_can::Packet& p) { to_receive.push(p); }

        std::queue<hi_can::Packet> to_receive;
        std::queue<hi_can::Packet> transmitted;
    };
}  // namespace hi_can_test

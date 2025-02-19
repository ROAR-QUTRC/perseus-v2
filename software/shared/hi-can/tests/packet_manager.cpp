#include <gtest/gtest.h>

#include <queue>
#include <vector>

#include "hi_can.hpp"

using namespace hi_can;

class FifoCanInterface : public CanInterface
{
public:
    void transmit(const Packet& packet) override
    {
        transmitQueue.push(packet);
    }
    std::optional<Packet> receive(bool blocking = false) override
    {
        (void)blocking;  // unused - we can't actually do anything with it
        if (receiveQueue.empty())
            return std::nullopt;
        Packet packet = receiveQueue.front();
        receiveQueue.pop();
        return packet;
    }

    std::queue<Packet> receiveQueue;
    std::queue<Packet> transmitQueue;
};

TEST(CanInterface, ReceiveAll)
{
    FifoCanInterface interface;
    Packet packet(addressing::flagged_address_t(0x12345678), std::vector<uint8_t>{0x01, 0x02, 0x03, 0x04});
    interface.receiveQueue.push(packet);
    interface.receiveAll();
    EXPECT_EQ(interface.receiveQueue.size(), 0);
}
#include <gtest/gtest.h>

#include <queue>

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

    void queueReceive(const Packet& packet)
    {
        receiveQueue.push(packet);
    }

    std::queue<Packet> receiveQueue;
    std::queue<Packet> transmitQueue;
};
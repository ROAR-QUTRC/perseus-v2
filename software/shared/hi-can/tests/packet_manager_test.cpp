#include <gtest/gtest.h>

#include <queue>

#include "hi_can.hpp"

using namespace hi_can;

class FifoCanInterface : public CanInterface
{
public:
    void transmit(const Packet& packet) override
    {
        transmit_queue.push(packet);
    }
    std::optional<Packet> receive(bool blocking = false) override
    {
        (void)blocking;  // unused - we can't actually do anything with it
        if (receive_queue.empty())
            return std::nullopt;
        Packet packet = receive_queue.front();
        receive_queue.pop();
        return packet;
    }

    void queue_receive(const Packet& packet)
    {
        receive_queue.push(packet);
    }

    std::queue<Packet> receive_queue;
    std::queue<Packet> transmit_queue;
};
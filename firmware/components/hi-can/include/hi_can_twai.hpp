#pragma once

#include "hi_can.hpp"

namespace hi_can
{
    class TwaiInterface : public CanInterface
    {
    public:
        TwaiInterface();
        ~TwaiInterface();

        TwaiInterface(const TwaiInterface&) = delete;
        TwaiInterface& operator=(const TwaiInterface&) = delete;

        void transmit(const Packet& packet) override;
        std::optional<Packet> receive(bool blocking = false) override;
        void receiveAll(bool block = true) override;
        void setReceiveCallback(const packet_callback_t& callback) override;
    };
}
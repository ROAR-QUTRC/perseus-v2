#pragma once

#include <chrono>
#include <functional>
#include <map>

#include "hi_can_address.hpp"
#include "hi_can_packet.hpp"

namespace hi_can
{
    /// @brief Interface for sending and receiving packets on the CAN bus
    class CanInterface
    {
    public:
        /// @brief Transmits a packet on the CAN bus
        /// @param packet Packet to transmit
        virtual void transmit(const Packet& packet) = 0;
        /// @brief Pulls the next packet from the CAN bus buffer
        /// @return The next packet in the buffer, or std::nullopt if the buffer is empty
        virtual std::optional<Packet> receive() = 0;

        virtual void setReceiveCallback(const std::function<void(const Packet&)>& callback) = 0;
    };

    /// @brief Manages automatically sending and receiving packets on the CAN bus with timeouts and callbacks
    class PacketManager
    {
    public:
        struct callback_config_t
        {
            can_address_t address = MAX_ADDRESS;
            std::function<void(const Packet&)> dataCallback;
            std::function<void(void)> timeoutCallback;
            std::function<void(const Packet&)> timeoutRecoveryCallback;
            std::chrono::steady_clock::duration timeout = std::chrono::steady_clock::duration::zero();
        };
        struct transmit_config_t
        {
            Packet packet;
            std::chrono::steady_clock::duration interval = std::chrono::steady_clock::duration::zero();
        };

        PacketManager(CanInterface& interface);

        void handle();

        void setCallback(const callback_config_t& config);
        std::optional<callback_config_t> getCallback(const can_address_t& address);
        void removeCallback(const can_address_t& address);

        void setTransmit(const transmit_config_t& config);
        void setTransmitData(const Packet& packet);
        void setTransmitInterval(const can_address_t& address, const std::chrono::steady_clock::duration& interval);
        void removeTransmit(const can_address_t& address);

        CanInterface& getInterface() { return _interface; }

    private:
        struct callback_data_t
        {
            callback_config_t config{};
            bool hasTimedOut = false;
            std::chrono::steady_clock::time_point lastReceived{};
        };
        struct transmit_data_t
        {
            transmit_config_t config{};
            std::chrono::steady_clock::time_point lastTransmitted{};
        };
        void _handlePacket(const Packet& packet);
        CanInterface& _interface;
        std::map<can_address_t, callback_data_t> _callbacks;
        std::map<can_address_t, transmit_data_t> _transmissions;
    };
};
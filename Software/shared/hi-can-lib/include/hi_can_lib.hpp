#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <set>

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
        virtual void transmit(const Packet& packet) const = 0;
        /// @brief Pulls the next packet from the CAN bus buffer
        /// @return The next packet in the buffer, or std::nullopt if the buffer is empty
        virtual std::optional<Packet> receive() = 0;

        /// @brief Sets a callback to be called when a packet is received
        /// @param callback Callback to use
        virtual void setReceiveCallback(const std::function<void(const Packet&)>& callback) = 0;
        /// @brief Clears the receive callback
        void clearReceiveCallback() { setReceiveCallback(nullptr); }
    };

    /// @brief A variant of @ref CanInterface with an address whitelist
    class FilteredCanInterface : public CanInterface
    {
    public:
        /// @brief Add an address to the whitelist
        /// @param address Address to add
        virtual void addAddress(const can_address_t& address) = 0;
        /// @brief Remove an address from the whitelist
        /// @param address Address to remove
        virtual void removeAddress(const can_address_t& address) = 0;

        /// @brief Add a range of addresses (inclusive) to the whitelist
        /// @param start Start address to add
        /// @param end End adress to add
        void addAddressRange(const can_address_t& start, const can_address_t& end);
        /// @brief Remove a range of addresses (inclusive) from the whitelist
        /// @param start Start address to remove
        /// @param end End adress to remove
        void removeAddressRange(const can_address_t& start, const can_address_t& end);

        /// @brief Add an iterable list of addresses to the whitelist
        /// @tparam Iterable Type of the iterable container
        /// @tparam C++ cursedness to ensure that the type is iterable
        /// @param addresses Addresses to add to the whitelist
        template <typename Iterable, typename = std::void_t<decltype(std::begin(std::declval<Iterable>())), decltype(std::end(std::declval<Iterable>()))>>
        void addAddressList(const Iterable& addresses)
        {
            for (const auto& address : addresses)
            {
                addAddress(address);
            }
        }

        /// @brief Remove an iterable list of addresses from the whitelist
        /// @tparam Iterable Type of the iterable container
        /// @tparam C++ cursedness to ensure that the type is iterable
        /// @param addresses Addresses to remove from the whitelist
        template <typename Iterable, typename = std::void_t<decltype(std::begin(std::declval<Iterable>())), decltype(std::end(std::declval<Iterable>()))>>
        void removeAddressList(const Iterable& addresses)
        {
            for (const auto& address : addresses)
            {
                removeAddress(address);
            }
        }
    };

    /// @brief A variant of @ref FilteredCanInterface implemented in software
    class SoftwareFilteredCanInterface : public FilteredCanInterface
    {
    public:
        SoftwareFilteredCanInterface(CanInterface& interface) : _interface(interface) {}

        void transmit(const Packet& packet) const override { _interface.transmit(packet); }
        std::optional<Packet> receive() override;
        void setReceiveCallback(const std::function<void(const Packet&)>& callback) override;

        void addAddress(const can_address_t& address) override { _whitelist.insert(address); }
        void removeAddress(const can_address_t& address) override { _whitelist.erase(address); }

    private:
        void _receiveCallback(const Packet& packet);
        CanInterface& _interface;
        std::set<can_address_t> _whitelist;
    };

    /// @brief Manages automatically sending and receiving packets on the CAN bus with timeouts and callbacks
    class PacketManager
    {
    public:
        struct callback_config_t
        {
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

        /// @brief Constructs a new PacketManager
        PacketManager(FilteredCanInterface& interface);

        void handle();

        void setCallback(const can_address_t& address, const callback_config_t& config);
        std::optional<callback_config_t> getCallback(const can_address_t& address);
        void removeCallback(const can_address_t& address);

        void setTransmit(const transmit_config_t& config);
        void setTransmitData(const Packet& packet);
        void setTransmitInterval(const can_address_t& address, const std::chrono::steady_clock::duration& interval);
        void removeTransmit(const can_address_t& address);

        FilteredCanInterface& getInterface() const { return _interface; }

    private:
        struct callback_data_t
        {
            callback_config_t config{};
            Packet lastPacket{};
            bool hasTimedOut = false;
            std::chrono::steady_clock::time_point lastReceived{};
        };
        struct transmit_data_t
        {
            transmit_config_t config{};
            std::chrono::steady_clock::time_point lastTransmitted{};
        };
        void _handleReceivedPacket(const Packet& packet);
        FilteredCanInterface& _interface;
        std::map<can_address_t, callback_data_t> _callbacks;
        std::map<can_address_t, transmit_data_t> _transmissions;
    };
};
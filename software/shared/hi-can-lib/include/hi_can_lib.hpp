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

        virtual void receiveAll();

        /// @brief Sets a callback to be called when a packet is received
        /// @param callback Callback to use
        virtual void setReceiveCallback(const std::function<void(const Packet&)>& callback) { _receiveCallback = callback; };
        /// @brief Clears the receive callback
        void clearReceiveCallback() { setReceiveCallback(nullptr); }

    protected:
        std::function<void(const Packet&)> _receiveCallback = nullptr;
    };

    /// @brief A variant of @ref CanInterface which contains a whitelist of filters. In the event that there are no filters, all packets are accepted.
    class FilteredCanInterface : public CanInterface
    {
    public:
        /// @brief Add a filter to the interface
        /// @return Itself for chaining
        virtual FilteredCanInterface& addFilter(const addressing::filter_t& address);
        /// @brief Remove a filter from the interface
        /// @return Itself for chaining
        virtual FilteredCanInterface& removeFilter(const addressing::filter_t& address);

        virtual const std::set<addressing::filter_t>& getFilters() const { return _filters; }

        virtual std::optional<addressing::filter_t> findMatchingFilter(const addressing::raw_address_t& address) const;
        virtual bool addressMatchesFilters(const addressing::raw_address_t& address) const;

    protected:
        friend void swap(FilteredCanInterface& first, FilteredCanInterface& second) noexcept
        {
            using std::swap;
            swap(first._filters, second._filters);
        }

        /// @brief List of currently applied filters
        std::set<addressing::filter_t> _filters;
    };
    void swap(FilteredCanInterface& first, FilteredCanInterface& second) noexcept;

    /// @brief A variant of @ref FilteredCanInterface implemented in software
    class SoftwareFilteredCanInterface : public FilteredCanInterface
    {
    public:
        SoftwareFilteredCanInterface(const std::shared_ptr<CanInterface> interface) : _interface(interface) {}

        void transmit(const Packet& packet) const override { _interface->transmit(packet); }
        std::optional<Packet> receive() override;

        // note: no need to override setReceiveCallback since that's based on the receive method

    private:
        const std::shared_ptr<CanInterface> _interface;
    };

    /// @brief Manages automatically sending and receiving packets on the CAN bus with timeouts and callbacks
    class PacketManager
    {
    public:
        struct callback_config_t
        {
            std::function<void(const Packet&)> dataCallback = nullptr;
            std::function<void(void)> timeoutCallback = nullptr;
            std::function<void(const Packet&)> timeoutRecoveryCallback = nullptr;
            std::chrono::steady_clock::duration timeout = std::chrono::steady_clock::duration::zero();
        };
        struct transmit_config_t
        {
            Packet packet;
            std::chrono::steady_clock::duration interval = std::chrono::steady_clock::duration::zero();
            bool shouldTransmitImmediately = false;
        };

        /// @brief Constructs a new PacketManager
        PacketManager(FilteredCanInterface& interface);

        void handle();

        void setCallback(const addressing::filter_t& address, const callback_config_t& config);
        std::optional<callback_config_t> getCallback(const addressing::filter_t& address);
        void removeCallback(const addressing::filter_t& address);

        void setTransmit(const transmit_config_t& config);
        void setTransmitData(const Packet& packet);
        void setTransmitInterval(const addressing::raw_address_t& address, const std::chrono::steady_clock::duration& interval);
        void removeTransmit(const addressing::raw_address_t& address);

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
        std::map<addressing::filter_t, callback_data_t> _callbacks;
        std::map<addressing::raw_address_t, transmit_data_t> _transmissions;
    };
};
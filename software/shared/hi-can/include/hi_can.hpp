#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <set>

#include "hi_can_address.hpp"
#include "hi_can_packet.hpp"

/// @brief Contains all classes, functions, and types related to implementing Hi-CAN
namespace hi_can
{
    // we use this type a lot so a typedef is convenient
    /// @brief A callback function which takes a @ref Packet as an argument and returns nothing
    typedef std::function<void(const hi_can::Packet&)> packet_callback_t;

    /// @brief Interface for sending and receiving packets on the CAN bus
    class CanInterface
    {
    public:
        /// @brief Transmits a packet on the CAN bus
        /// @param packet Packet to transmit
        virtual void transmit(const Packet& packet) = 0;
        /// @brief Pulls the next packet from the CAN bus buffer
        /// @param blocking Whether to block until a packet is received
        /// @return The next packet in the buffer, or std::nullopt if the buffer is empty
        virtual std::optional<Packet> receive(bool blocking = false) = 0;

        /// @brief Receives all packets from the CAN bus buffer
        /// @param block Whether to block until at least one packet is received
        virtual void receiveAll(bool block = true);

        /// @brief Sets a callback to be called when a packet is received
        /// @param callback Callback to use
        virtual void setReceiveCallback(const packet_callback_t& callback) { _receiveCallback = callback; };
        /// @brief Clears the receive callback
        void clearReceiveCallback() { setReceiveCallback(nullptr); }

    protected:
        /// @brief The callback to call when a packet is received
        packet_callback_t _receiveCallback = nullptr;
    };

    /// @brief A variant of @ref CanInterface which contains a whitelist of filters. In the event that there are no filters, all packets are accepted.
    class FilteredCanInterface : public CanInterface
    {
    public:
        /// @brief Add a filter to the interface
        /// @return Itself for chaining
        virtual FilteredCanInterface& addFilter(const addressing::filter_t& filter);
        /// @brief Remove a filter from the interface
        /// @return Itself for chaining
        virtual FilteredCanInterface& removeFilter(const addressing::filter_t& filter);

        /// @brief Get the currently active filters
        /// @return The set of active filters
        virtual const std::set<addressing::filter_t>& getFilters() const { return _filters; }

        /// @brief Find the first filter which matches the given address
        /// @param address The address to search for
        /// @return The matching filter if found, otherwise std::nullopt
        virtual std::optional<addressing::filter_t> findMatchingFilter(const addressing::flagged_address_t& address) const;
        /// @brief  Check if an address matches any of the filters
        /// @param address The address to check
        /// @return Whether the address matches a filter
        virtual bool addressMatchesFilters(const addressing::flagged_address_t& address) const;

    protected:
        /// @brief List of currently applied filters
        std::set<addressing::filter_t> _filters;
    };

    /// @brief A variant of @ref FilteredCanInterface implemented in software
    class SoftwareFilteredCanInterface : public FilteredCanInterface
    {
    public:
        /// @brief Constructs a new @ref SoftwareFilteredCanInterface using the given interface for I/O
        /// @param interface The interface to use for I/O
        SoftwareFilteredCanInterface(const std::shared_ptr<CanInterface> interface) : _interface(interface) {}

        // note: docs inherited from base class
        void transmit(const Packet& packet) override { _interface->transmit(packet); }
        std::optional<Packet> receive(bool blocking = false) override;

        // note: no need to override setReceiveCallback since that's based on the receive method

    private:
        /// @brief The interface to use for I/O
        const std::shared_ptr<CanInterface> _interface;
    };

    /// @brief Manages automatically sending and receiving packets on the CAN bus with timeouts and callbacks
    class PacketManager
    {
    public:
        /// @brief Configuration for a callback
        struct callback_config_t
        {
            /// @brief The callback to be called when data is received
            packet_callback_t dataCallback = nullptr;
            /// @brief The callback to be called when a timeout occurs
            std::function<void(void)> timeoutCallback = nullptr;
            /// @brief The callback to be called when data is received after a timeout
            packet_callback_t timeoutRecoveryCallback = nullptr;
            /// @brief The timeout duration. Zero means no timeout
            std::chrono::steady_clock::duration timeout = std::chrono::steady_clock::duration::zero();
        };
        /// @brief Configuration for scheduling a packet transmission
        struct transmit_config_t
        {
            /// @brief The packet to transmit
            Packet packet;
            /// @brief The interval between transmissions - zero means transmit once, regardless of @ref shouldTransmitImmediately
            std::chrono::steady_clock::duration interval = std::chrono::steady_clock::duration::zero();
            /// @brief Whether or not to transmit the packet immediately. If false, will transmit after the interval
            bool shouldTransmitImmediately = false;
        };

        /// @brief Constructs a new @ref PacketManager using the given interface for I/O
        /// @param interface The interface to use for I/O
        PacketManager(FilteredCanInterface& interface);

        /// @brief Handles all callbacks and transmissions
        void handle();

        /// @brief Sets a data receive callback which will be called for packets received on the interface matching the filter
        /// @param filter The filter to match packets against
        /// @param config The configuration for the callback
        /// @note The filter will be added to the interface's receive filter list
        void setCallback(const addressing::filter_t& filter, const callback_config_t& config);
        /// @brief Get the callback configuration for a specific filter
        /// @param filter The filter to get the configuration for
        /// @return The callback configuration if found, otherwise std::nullopt
        std::optional<callback_config_t> getCallback(const addressing::filter_t& filter);
        /// @brief Remove a callback for a specific filter
        /// @param filter The filter to remove the callback for
        /// @note The filter will be removed from the interface's receive filter list
        void removeCallback(const addressing::filter_t& filter);

        /// @brief Set a transmit configuration
        /// @param config The configuration to set
        void setTransmitConfig(const transmit_config_t& config);
        /// @brief Overwrite the transmit data for a transmit configuration
        /// @param packet The packet to transmit
        /// @note If there is no transmit configuration for the packet's address, nothing will happen
        void setTransmitData(const Packet& packet);
        /// @brief Set the transmission interval for an address
        /// @param address The address to set the interval for
        /// @param interval New transmission interval
        /// @note If there is no transmit configuration for the address, nothing will happen
        void setTransmitInterval(const addressing::flagged_address_t& address, const std::chrono::steady_clock::duration& interval);
        /// @brief Get the transmit configuration for an address
        /// @param address The address to get the configuration for
        /// @return The transmit configuration if found, otherwise std::nullopt
        std::optional<transmit_config_t> getTransmitConfig(const addressing::flagged_address_t& address);
        /// @brief Remove a transmit configuration
        /// @param address The address to remove the configuration for
        void removeTransmit(const addressing::flagged_address_t& address);

        /// @brief Get the underlying interface used for I/O
        /// @return The interface
        FilteredCanInterface& getInterface() const { return _interface; }

    private:
        /// @brief Struct storing all the data we need to track to handle RX callbacks
        struct callback_data_t
        {
            /// @brief The callback config
            callback_config_t config{};
            /// @brief The last packet received
            Packet lastPacket{};
            /// @brief Whether or not it's currently timed out
            bool hasTimedOut = false;
            /// @brief The last time a packet was received
            std::chrono::steady_clock::time_point lastReceived{};
        };
        /// @brief Struct storing all the data we need to track to handle transmissions
        struct transmit_data_t
        {
            /// @brief The transmit configuration
            transmit_config_t config{};
            /// @brief The last time the data was transmitted
            std::chrono::steady_clock::time_point lastTransmitted{};
        };
        /// @brief Handle an incoming packet and call the correct callbacks
        /// @param packet The packet to handle
        void _handleReceivedPacket(const Packet& packet);
        /// @brief The underlying I/O interface
        FilteredCanInterface& _interface;
        /// @brief Map of filters to their callback data
        std::map<addressing::filter_t, callback_data_t> _callbacks;
        /// @brief Map of addresses to their transmit data
        std::map<addressing::flagged_address_t, transmit_data_t> _transmissions;
    };
};
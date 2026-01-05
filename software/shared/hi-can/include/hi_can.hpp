#pragma once

#include <set>

#include "hi_can_address.hpp"
#include "hi_can_packet.hpp"
#include "hi_can_parameter.hpp"

/// @brief Contains all classes, functions, and types related to implementing Hi-CAN
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
        /// @param blocking Whether to block until a packet is received
        /// @return The next packet in the buffer, or std::nullopt if the buffer is empty
        virtual std::optional<Packet> receive(bool blocking = false) = 0;

        /// @brief Receives all packets from the CAN bus buffer
        /// @param block Whether to block until at least one packet is received
        virtual void receive_all(bool block = true);

        /// @brief Sets a callback to be called when a packet is received
        /// @param callback Callback to use
        virtual void set_receive_callback(const packet_callback_t& callback) { _receive_callback = callback; };
        /// @brief Clears the receive callback
        void clear_receive_callback() { set_receive_callback(nullptr); }

    protected:
        /// @brief The callback to call when a packet is received
        packet_callback_t _receive_callback = nullptr;
    };

    /// @brief A variant of @ref CanInterface which contains a whitelist of filters. In the event that there are no filters, all packets are accepted.
    class FilteredCanInterface : public CanInterface
    {
    public:
        /// @brief Add a filter to the interface
        /// @return Itself for chaining
        virtual FilteredCanInterface& add_filter(const addressing::filter_t& filter);
        /// @brief Remove a filter from the interface
        /// @return Itself for chaining
        virtual FilteredCanInterface& remove_filter(const addressing::filter_t& filter);

        /// @brief Get the currently active filters
        /// @return The set of active filters
        virtual const std::set<addressing::filter_t>& get_filters() const { return _filters; }

        /// @brief Find the first filter which matches the given address
        /// @param address The address to search for
        /// @return The matching filter if found, otherwise std::nullopt
        virtual std::optional<addressing::filter_t> find_matching_filter(const addressing::flagged_address_t& address) const;
        /// @brief  Check if an address matches any of the filters
        /// @param address The address to check
        /// @return Whether the address matches a filter
        virtual bool address_matches_filters(const addressing::flagged_address_t& address) const;

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
        SoftwareFilteredCanInterface(const std::shared_ptr<CanInterface> interface)
            : _interface(interface)
        {
        }

        // note: docs inherited from base class
        void transmit(const Packet& packet) override { _interface->transmit(packet); }
        std::optional<Packet> receive(bool blocking = false) override;

        // note: no need to override set_receive_callback since that's based on the receive method

    private:
        /// @brief The interface to use for I/O
        const std::shared_ptr<CanInterface> _interface;
    };
};
#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <map>
#include <optional>
#include <stdexcept>
#include <vector>

#include "hi_can_address.hpp"

namespace hi_can
{
    /// @brief A CAN packet containing an address and data
    class Packet
    {
    public:
        /// @brief Default constructor - zero address, no data
        Packet() = default;

        /// @brief Construct a packet from an address and data array
        /// @param address The address of the packet
        /// @param data The packet data
        /// @param data_len The length of the data array
        Packet(const addressing::flagged_address_t& address, const uint8_t data[] = nullptr, size_t data_len = 0);
        /// @brief Construct a packet from an address and data vector
        /// @param address The address of the packet
        /// @param data The data vector
        Packet(const addressing::flagged_address_t& address, const std::vector<uint8_t>& data);
        /// @brief Construct a packet from an address and data
        /// @tparam T The type of the data
        /// @param address The address of the packet
        /// @param data The data to copy into the packet
        template <typename T>
        Packet(const addressing::flagged_address_t& address, const T& data)
        {
            set_address(address);
            set_data(data);
        }

        /// @brief Copies a packet's data into a buffer
        /// @tparam T The type of the buffer to copy it into
        /// @return The buffer with the data copied into it, or std::nullopt if the buffer does not match the packet size
        template <typename T>
        std::optional<T> get_data() const
        {
            if (_data.size() != sizeof(T))
                return std::nullopt;

            T data;
            std::copy(_data.begin(), _data.end(), reinterpret_cast<uint8_t* const>(&data));
            return data;
        }
        /// @brief Get the packet data
        /// @return The packet data
        constexpr const auto& get_data() const { return _data; }

        /// @brief Set the packet data
        /// @param data The data to copy into the packet
        /// @param data_len The length of the data array
        /// @exception std::invalid_argument If the data is too large to fit in the packet
        void set_data(const uint8_t data[], size_t data_len);
        /// @brief Set the packet data
        /// @param data The data to copy into the packet
        /// @exception std::invalid_argument If the data is too large to fit in the packet
        void set_data(const std::vector<uint8_t>& data);
        /// @brief Set the packet data
        /// @tparam T The type of the data
        /// @param data The data to copy into the packet
        /// @exception std::invalid_argument If the data is too large to fit in the packet
        template <typename T>
        void set_data(const T& data)
        {
            set_data(reinterpret_cast<const uint8_t* const>(&data), sizeof(T));
        }

        /// @brief Get the length of the data in the packet
        /// @return The data length
        constexpr auto get_data_len() const { return _data.size(); }

        /// @brief Get the packet address
        /// @return The packet address
        constexpr auto get_address() const { return _address; }
        /// @brief Set the packet address
        /// @param address The address to set
        void set_address(const addressing::flagged_address_t& address);

        /// @brief Get whether the packet address is RTR
        /// @return Whether the address is RTR
        constexpr bool get_is_rtr() const { return _address.is_rtr; }
        /// @brief Set whether the packet address is RTR
        /// @param is_rtr Whether the address is RTR
        void set_is_rtr(const bool& is_rtr) { _address.is_rtr = is_rtr; }

        /// @brief Get whether the packet address is an error
        /// @return Whether the address is an error
        constexpr bool get_is_error() const { return _address.is_error; }
        /// @brief Set whether the packet address is an error
        /// @param is_error Whether the address is an error
        void set_is_error(const bool& is_error) { _address.is_error = is_error; }

        /// @brief Get whether the packet address is extended
        /// @return Whether the address is extended
        constexpr bool get_is_extended() const { return _address.is_extended; }
        /// @brief Set whether the packet address is extended
        /// @param is_extended Whether the address is extended
        void set_is_extended(const bool& is_extended) { _address.is_extended = is_extended; }

        // implement comparison functions for STL containers
        /// @brief Compare two packets (for sorting). Sorts only by @ref addressing::flagged_address_t "address"
        /// @param other Packet to compare against
        constexpr auto operator<=>(const Packet& other) const { return _address <=> other._address; }
        /// @brief Check if two packets are equal. Checks address and data.
        /// @param other Packet to compare against
        /// @return Whether the packets are equal
        constexpr auto operator==(const Packet& other) const
        {
            return (_address == other._address) &&
                   (_data == other._data);
        }
        /// @brief Check if two packets are not equal
        /// @param other Packet to compare against
        /// @return Whether the packets differ
        constexpr auto operator!=(const Packet& other) const { return !(*this == other); }

    private:
        /// @brief The packet address
        addressing::flagged_address_t _address = addressing::MAX_ADDRESS;
        /// @brief The packet data
        std::vector<uint8_t> _data{};
    };

    /// @brief A callback function which takes a @ref Packet as an argument and returns nothing
    typedef std::function<void(const hi_can::Packet&)> packet_callback_t;

    // forward declarations to avoid circular dependencies
    namespace parameters
    {
        class ParameterGroup;
    }
    class FilteredCanInterface;
    /// @brief Manages automatically sending and receiving packets on the CAN bus with timeouts and callbacks
    class PacketManager
    {
    public:
        /// @brief Configuration for a data reception callback
        struct callback_config_t
        {
            /// @brief The callback to be called when data is received
            packet_callback_t data_callback = nullptr;
            /// @brief The callback to be called when a timeout occurs
            std::function<void(void)> timeout_callback = nullptr;
            /// @brief The callback to be called when data is received after a timeout
            packet_callback_t timeout_recovery_callback = nullptr;
            /// @brief The timeout duration. Zero means no timeout
            std::chrono::steady_clock::duration timeout = std::chrono::steady_clock::duration::zero();
        };
        /// @brief Function which generates packet data
        typedef std::function<std::vector<uint8_t>(void)> data_generator_t;
        /// @brief Configuration for scheduling a packet transmission
        struct transmission_config_t
        {
            /// @brief Function which returns the packet data to transmit
            data_generator_t generator = nullptr;
            /// @brief The interval between transmissions - zero means transmit as often as possible
            std::chrono::steady_clock::duration interval = std::chrono::steady_clock::duration::zero();
            /// @brief Whether or not to transmit the packet immediately. If false, will transmit after the interval
            bool should_transmit_immediately = false;
        };

        /// @brief Constructs a new @ref PacketManager using the given interface for I/O
        /// @param interface The interface to use for I/O
        PacketManager(FilteredCanInterface& interface);

        /// @brief Handles all data reception and transmission - just calls @ref handle_receive and @ref handle_transmit
        /// @param should_block Whether or not to block until a packet is received
        /// @param should_force_transmission Force transmission of all packets, even if they are not due
        void handle(bool should_block = false, bool should_force_transmission = false)
        {
            handle_receive(should_block);
            handle_transmit(should_force_transmission);
        }
        /// @brief Handles all data reception and associated callbacks
        /// @param should_block Whether or not to block until a packet is received
        void handle_receive(bool should_block = false);
        /// @brief Handles all data transmissions
        /// @param should_force_transmission Force transmission of all packets, even if they are not due
        void handle_transmit(bool should_force_transmission = false);

        /// @brief Add a parameter group to the packet manager
        /// @param group Group to add
        void add_group(const parameters::ParameterGroup& group);

        /// @brief Remove a parameter group from the packet manager
        /// @param group Group to remove
        void remove_group(const parameters::ParameterGroup& group);

        /// @brief Sets a data receive callback which will be called for packets received on the interface matching the filter
        /// @param filter The filter to match packets against
        /// @param config The configuration for the callback
        /// @note The filter will be added to the interface's receive filter list
        void set_callback(const addressing::filter_t& filter, const callback_config_t& config);
        /// @brief Get the callback configuration for a specific filter
        /// @param filter The filter to get the configuration for
        /// @return The callback configuration if found, otherwise std::nullopt
        std::optional<callback_config_t> get_callback(const addressing::filter_t& filter);
        /// @brief Remove a callback for a specific filter
        /// @param filter The filter to remove the callback for
        /// @note The filter will be removed from the interface's receive filter list
        void remove_callback(const addressing::filter_t& filter);

        /// @brief Set a transmit configuration
        /// @param config The configuration to set
        void set_transmission_config(const addressing::flagged_address_t& address, const transmission_config_t& config);
        /// @brief Overwrite the transmission data generator for an address
        /// @param address The address to set the generator for
        /// @param generator The transmission data generator
        /// @note If there is no static transmit configuration for the packet's address, nothing will happen
        void set_transmission_generator(const addressing::flagged_address_t& address, const data_generator_t& generator);
        /// @brief Set the transmission interval for an address
        /// @param address The address to set the interval for
        /// @param interval New transmission interval
        /// @note If there is no transmit configuration for the address, nothing will happen
        void set_transmission_interval(const addressing::flagged_address_t& address, const std::chrono::steady_clock::duration& interval);
        /// @brief Get the transmit configuration for an address
        /// @param address The address to get the configuration for
        /// @return The transmit configuration if found, otherwise std::nullopt
        std::optional<transmission_config_t> get_transmission_config(const addressing::flagged_address_t& address);
        /// @brief Remove a transmit configuration
        /// @param address The address to remove the configuration for
        void remove_transmission(const addressing::flagged_address_t& address);

        /// @brief Get the underlying interface used for I/O
        /// @return The interface
        FilteredCanInterface& get_interface() const { return _interface; }

    private:
        /// @brief Struct storing all the data we need to track to handle RX callbacks
        struct callback_data_t
        {
            /// @brief The callback config
            callback_config_t config{};
            /// @brief The last packet received
            Packet last_packet{};
            /// @brief Whether or not it's currently timed out
            bool has_timed_out = false;
            /// @brief The last time a packet was received
            std::chrono::steady_clock::time_point last_received{};
        };
        /// @brief Struct storing all the data we need to track to handle transmissions
        struct transmission_data_t
        {
            /// @brief The transmit configuration
            transmission_config_t config{};
            /// @brief The last time the data was transmitted
            std::chrono::steady_clock::time_point last_transmitted{};
        };
        /// @brief Handle an incoming packet and call the correct callbacks
        /// @param packet The packet to handle
        void _handle_received_packet(const Packet& packet);
        /// @brief The underlying I/O interface
        FilteredCanInterface& _interface;
        /// @brief Map of filters to their callback data
        std::map<addressing::filter_t, callback_data_t> _callbacks;
        /// @brief Map of addresses to their transmit data
        std::map<addressing::flagged_address_t, transmission_data_t> _transmissions;
    };
}
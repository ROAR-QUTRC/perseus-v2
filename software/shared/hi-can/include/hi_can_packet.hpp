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
        /// @brief Default constructor
        /// @param address THe address of the packet
        /// @note Address is set to MAX_ADDRESS by default to make it very obvious that the packet is invalid if it gets transmitted
        explicit Packet(const addressing::flagged_address_t& address = addressing::MAX_ADDRESS) : Packet(address, nullptr, 0) {}

        /// @brief Construct a packet from an address and data array
        /// @param address The address of the packet
        /// @param data The packet data
        /// @param dataLen The length of the data array
        Packet(const addressing::flagged_address_t& address, const uint8_t data[], size_t dataLen);
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
            setAddress(address);
            setData(data);
        }

        /// @brief Copies a packet's data into a buffer
        /// @tparam T The type of the buffer to copy it into
        /// @return The buffer with the data copied into it, or std::nullopt if the buffer does not match the packet size
        template <typename T>
        std::optional<T> getData() const
        {
            if (_data.size() != sizeof(T))
                return std::nullopt;

            T data;
            std::copy(_data.begin(), _data.end(), reinterpret_cast<uint8_t* const>(&data));
            return data;
        }
        /// @brief Get the packet data
        /// @return The packet data
        constexpr const auto& getData() const { return _data; }

        /// @brief Set the packet data
        /// @param data The data to copy into the packet
        /// @param dataLen The length of the data array
        /// @exception std::invalid_argument If the data is too large to fit in the packet
        void setData(const uint8_t data[], size_t dataLen);
        /// @brief Set the packet data
        /// @param data The data to copy into the packet
        /// @exception std::invalid_argument If the data is too large to fit in the packet
        void setData(const std::vector<uint8_t>& data);
        /// @brief Set the packet data
        /// @tparam T The type of the data
        /// @param data The data to copy into the packet
        /// @exception std::invalid_argument If the data is too large to fit in the packet
        template <typename T>
        void setData(const T& data)
        {
            setData(reinterpret_cast<const uint8_t* const>(&data), sizeof(T));
        }

        /// @brief Get the length of the data in the packet
        /// @return The data length
        constexpr auto getDataLen() const { return _data.size(); }

        /// @brief Get the packet address
        /// @return The packet address
        constexpr auto getAddress() const { return _address; }
        /// @brief Set the packet address
        /// @param address The address to set
        void setAddress(const addressing::flagged_address_t& address);

        /// @brief Get whether the packet address is RTR
        /// @return Whether the address is RTR
        constexpr bool getIsRTR() const { return _address.isRtr; }
        /// @brief Set whether the packet address is RTR
        /// @param isRTR Whether the address is RTR
        void setIsRTR(const bool& isRTR) { _address.isRtr = isRTR; }

        /// @brief Get whether the packet address is an error
        /// @return Whether the address is an error
        constexpr bool getIsError() const { return _address.isError; }
        /// @brief Set whether the packet address is an error
        /// @param isError Whether the address is an error
        void setIsError(const bool& isError) { _address.isError = isError; }

        /// @brief Get whether the packet address is extended
        /// @return Whether the address is extended
        constexpr bool getIsExtended() const { return _address.isExtended; }
        /// @brief Set whether the packet address is extended
        /// @param isExtended Whether the address is extended
        void setIsExtended(const bool& isExtended) { _address.isExtended = isExtended; }

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
        addressing::flagged_address_t _address{};
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
            packet_callback_t dataCallback = nullptr;
            /// @brief The callback to be called when a timeout occurs
            std::function<void(void)> timeoutCallback = nullptr;
            /// @brief The callback to be called when data is received after a timeout
            packet_callback_t timeoutRecoveryCallback = nullptr;
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
            bool shouldTransmitImmediately = false;
        };

        /// @brief Constructs a new @ref PacketManager using the given interface for I/O
        /// @param interface The interface to use for I/O
        PacketManager(FilteredCanInterface& interface);

        /// @brief Handles all data reception and transmission - just calls @ref handleReceive and @ref handleTransmit
        /// @param shouldBlock Whether or not to block until a packet is received
        /// @param shouldForceTransmission Force transmission of all packets, even if they are not due
        void handle(bool shouldBlock = false, bool shouldForceTransmission = false)
        {
            handleReceive(shouldBlock);
            handleTransmit(shouldForceTransmission);
        }
        /// @brief Handles all data reception and associated callbacks
        /// @param shouldBlock Whether or not to block until a packet is received
        void handleReceive(bool shouldBlock = false);
        /// @brief Handles all data transmissions
        /// @param shouldForceTransmission Force transmission of all packets, even if they are not due
        void handleTransmit(bool shouldForceTransmission = false);

        /// @brief Add a parameter group to the packet manager
        /// @param group Group to add
        void addGroup(const parameters::ParameterGroup& group);

        /// @brief Remove a parameter group from the packet manager
        /// @param group Group to remove
        void removeGroup(const parameters::ParameterGroup& group);

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
        void setTransmissionConfig(const addressing::flagged_address_t& address, const transmission_config_t& config);
        /// @brief Overwrite the transmission data generator for an address
        /// @param address The address to set the generator for
        /// @param generator The transmission data generator
        /// @note If there is no static transmit configuration for the packet's address, nothing will happen
        void setTransmissionGenerator(const addressing::flagged_address_t& address, const data_generator_t& generator);
        /// @brief Set the transmission interval for an address
        /// @param address The address to set the interval for
        /// @param interval New transmission interval
        /// @note If there is no transmit configuration for the address, nothing will happen
        void setTransmissionInterval(const addressing::flagged_address_t& address, const std::chrono::steady_clock::duration& interval);
        /// @brief Get the transmit configuration for an address
        /// @param address The address to get the configuration for
        /// @return The transmit configuration if found, otherwise std::nullopt
        std::optional<transmission_config_t> getTransmissionConfig(const addressing::flagged_address_t& address);
        /// @brief Remove a transmit configuration
        /// @param address The address to remove the configuration for
        void removeTransmission(const addressing::flagged_address_t& address);

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
        struct transmission_data_t
        {
            /// @brief The transmit configuration
            transmission_config_t config{};
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
        std::map<addressing::flagged_address_t, transmission_data_t> _transmissions;
    };
}
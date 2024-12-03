#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <stdexcept>

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
        /// @param dataLen The length of the data array
        Packet(const addressing::flagged_address_t& address, const uint8_t data[], size_t dataLen);
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
            if (_dataLen != sizeof(T))
                return std::nullopt;

            T data;
            std::copy_n(_data.begin(), _dataLen, reinterpret_cast<uint8_t* const>(&data));
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
        constexpr auto getDataLen() const { return _dataLen; }

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
                   (_dataLen == other._dataLen) &&
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
        std::array<uint8_t, addressing::MAX_PACKET_LEN> _data{};
        /// @brief The length of the data in the packet
        size_t _dataLen = 0;
    };
}
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <stdexcept>

#include "hi_can_address.hpp"

namespace hi_can
{
    class Packet
    {
    public:
        Packet() = default;

        Packet(const address::flagged_address_t& address, const uint8_t data[], size_t dataLen);
        template <typename T>
        Packet(const address::flagged_address_t& address, const T& data)
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
        constexpr const auto& getData() const { return _data; }

        void setData(const uint8_t data[], size_t dataLen);
        template <typename T>
        void setData(const T& data)
        {
            setData(reinterpret_cast<const uint8_t* const>(&data), sizeof(T));
        }

        constexpr auto getDataLen() const { return _dataLen; }

        constexpr auto getAddress() const { return _address; }
        void setAddress(const address::flagged_address_t& address);

        constexpr bool getIsRTR() const { return _address.rtr; }
        void setIsRTR(const bool& isRTR) { _address.rtr = isRTR; }

        constexpr bool getIsError() const { return _address.error; }
        void setIsError(const bool& isError) { _address.error = isError; }

        constexpr bool getIsExtended() const { return _address.extended; }
        void setIsExtended(const bool& isExtended) { _address.extended = isExtended; }

        // implement comparison functions for STL containers - comparison based on address
        constexpr auto operator<=>(const Packet& other) const { return _address <=> other._address; }
        constexpr auto operator==(const Packet& other) const
        {
            return (_address == other._address) &&
                   (_dataLen == other._dataLen) &&
                   (_data == other._data);
        }
        constexpr auto operator!=(const Packet& other) const { return !(*this == other); }

    private:
        address::flagged_address_t _address = address::MAX_ADDRESS;
        std::array<uint8_t, address::MAX_PACKET_LEN> _data{};
        size_t _dataLen = 0;
    };
}
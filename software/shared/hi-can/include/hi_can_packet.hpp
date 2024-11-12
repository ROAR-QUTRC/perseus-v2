#pragma once

#include <algorithm>
#include <array>
#include <optional>
#include <stdexcept>

#include "hi_can_address.hpp"

namespace hi_can
{
    class Packet
    {
    public:
        Packet() = default;

        Packet(const address::raw_address_t& address, const uint8_t data[], size_t dataLen, const bool& isRTR = false);
        template <typename T>
        Packet(const address::raw_address_t& address, const T& data, const bool& isRTR = false)
        {
            setAddress(address);
            setIsRTR(isRTR);
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
        void setAddress(const address::raw_address_t& address);

        constexpr bool getIsRTR() const { return _isRTR; }
        void setIsRTR(const bool& isRTR) { _isRTR = isRTR; }

        constexpr bool dataEquals(const Packet& other) const
        {
            return (_address == other._address) &&
                   (_dataLen == other._dataLen) &&
                   (_data == other._data) &&
                   (_isRTR == other._isRTR);
        }

        // implement comparison functions for STL containers - comparison based on address
        constexpr bool operator<(const Packet& other) const { return _address < other._address; }
        constexpr bool operator>(const Packet& other) const { return _address > other._address; }
        constexpr bool operator==(const Packet& other) const { return _address == other._address; }

    private:
        address::raw_address_t _address = address::MAX_ADDRESS;
        std::array<uint8_t, address::MAX_PACKET_LEN> _data{};
        size_t _dataLen = 0;
        bool _isRTR = false;
    };
}
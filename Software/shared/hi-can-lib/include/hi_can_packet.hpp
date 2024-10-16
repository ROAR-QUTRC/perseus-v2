#pragma once

#include <memory>
#include <optional>

#include "hi_can_address.hpp"

namespace hi_can
{
    class Packet
    {
    public:
        Packet() = default;
        Packet(const can_address_t& address, const std::unique_ptr<void>& data, const size_t& dataLen, const bool& isRTR = false);
        template <typename T>
        Packet(const can_address_t& address, const T& data, const bool& isRTR = false)
            : Packet(address, std::make_unique<T>(data), sizeof(T), isRTR)
        {
        }

        /// @brief Copys a packet's data into a buffer
        /// @tparam T The type of the buffer to copy it into
        /// @return The buffer with the data copied into it, or std::nullopt if the buffer does not match the packet size
        template <typename T>
        std::optional<T> getData() const
        {
            if (_dataLen != sizeof(T))
                return std::nullopt;

            T data;
            memcpy(&data, _data, sizeof(T));
            return data;
        }
        const uint8_t* getData() const { return _data; }

        void setData(const std::unique_ptr<void>& data, const size_t& dataLen);
        template <typename T>
        void setData(const T& data)
        {
            setData(std::make_unique<T>(data), sizeof(T));
        }

        uint8_t getDataLen() const { return _dataLen; }

        const can_address_t& getAddress() const { return _address; }
        void setAddress(const can_address_t& address) { _address = address; }

        bool getIsRTR() const { return _isRTR; }
        void setIsRTR(const bool& isRTR) { _isRTR = isRTR; }

    private:
        uint8_t _data[MAX_PACKET_LEN]{};
        uint8_t _dataLen = 0;
        can_address_t _address = MAX_ADDRESS;
        bool _isRTR = false;
    };
}
#include "hi_can_packet.hpp"

#include <cstring>

using namespace hi_can;

Packet::Packet(const can_address_t& address, const std::unique_ptr<void>& data, const size_t& dataLen, const bool& isRTR)
    : _dataLen(dataLen), _address(address), _isRTR(isRTR)
{
    if (address > MAX_ADDRESS)
        throw std::invalid_argument("Address is invalid");

    setData(data, dataLen);
}

void Packet::setData(const std::unique_ptr<void>& data, const size_t& dataLen)
{
    {
        if (dataLen > MAX_PACKET_LEN)
            throw std::invalid_argument("Data is too long");
        if (dataLen > 0)
        {
            if (!data)
                throw std::invalid_argument("Data is null");
        }

        _dataLen = dataLen;
        memcpy(_data, data.get(), dataLen);
    }
}
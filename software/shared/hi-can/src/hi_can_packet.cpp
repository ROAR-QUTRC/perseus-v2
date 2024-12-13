#include "hi_can_packet.hpp"

using namespace hi_can;

Packet::Packet(const addressing::flagged_address_t& address, const uint8_t data[], size_t dataLen)
{
    setData(data, dataLen);
    setAddress(address);
}

void Packet::setData(const uint8_t data[], size_t dataLen)
{
    if (dataLen > addressing::MAX_PACKET_LEN)
        throw std::invalid_argument("Data is longer than the maximum packet length");

    _dataLen = dataLen;
    std::copy_n(data, dataLen, _data.begin());
}

void Packet::setAddress(const addressing::flagged_address_t& address)
{
    if (address > addressing::MAX_ADDRESS)
        throw std::invalid_argument("Address is invalid");
    _address = address;
}
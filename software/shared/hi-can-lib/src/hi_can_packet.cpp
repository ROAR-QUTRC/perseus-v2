#include "hi_can_packet.hpp"

using namespace hi_can;

Packet::Packet(const addressing::raw_address_t& address, const std::span<const uint8_t>& data, const bool& isRTR)
{
    setAddress(address);
    setIsRTR(isRTR);
    if (data.size_bytes() > addressing::MAX_PACKET_LEN)
        throw std::invalid_argument("Data is too long");
    std::copy_n(data.begin(), data.size_bytes(), _data.begin());
}

void Packet::setAddress(const addressing::raw_address_t& address)
{
    if (address > addressing::MAX_ADDRESS)
        throw std::invalid_argument("Address is invalid");
    _address = address;
}
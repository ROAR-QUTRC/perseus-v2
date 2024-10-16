#include "hi_can_raw.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>

using namespace hi_can;
using std::string;

RawCanInterface::RawCanInterface(string interfaceName)
    : _interfaceName(interfaceName)
{
    // Create CAN socket
    _socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (_socket < 0)
    {
        throw std::runtime_error("Failed to create CAN socket");
    }

    // Get the index of the network interface
    int interface_idx = 0;
    if (interfaceName != "any")
    {
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, _interfaceName.c_str(), IFNAMSIZ - 1);
        if (ioctl(_socket, SIOCGIFINDEX, &ifr) < 0)
        {
            close(_socket);
            throw std::runtime_error("Failed to get interface index");
        }
    }

    // Bind socket to the specified CAN interface
    struct sockaddr_can addr
    {
    };
    addr.can_family = AF_CAN;
    addr.can_ifindex = interface_idx;
    if (bind(_socket, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        close(_socket);
        throw std::runtime_error("Failed to bind CAN socket");
    }
}

RawCanInterface::~RawCanInterface()
{
    // no need for error checking, constructor would have thrown an exception
    close(_socket);
}

RawCanInterface::RawCanInterface(const RawCanInterface& other)
    : RawCanInterface(other._interfaceName)
{
}

void RawCanInterface::transmit(const Packet& packet) const
{
    struct can_frame frame
    {
    };

    // set CAN ID, as well as necessary flags
    frame.can_id = packet.getAddress() | CAN_EFF_FLAG;  // use 29-bit extended frame format
    if (packet.getIsRTR())
    {
        frame.can_id |= CAN_RTR_FLAG;
    }

    frame.len = packet.getDataLen();
    if (frame.len > CAN_MAX_DLEN)
    {
        throw std::runtime_error("Packet data length exceeds maximum CAN frame length");
    }
    if (packet.getData() == nullptr)
    {
        throw std::runtime_error("Packet data is null");
    }
    memcpy(frame.data, packet.getData(), frame.len);

    if (int bytesWritten = write(_socket, &frame, sizeof(frame)); bytesWritten < 0)
    {
        throw std::runtime_error("Failed to transmit CAN frame");
    }
    else if (bytesWritten != frame.len)
    {
        throw std::runtime_error("Failed to transmit entire CAN frame");
    }
}
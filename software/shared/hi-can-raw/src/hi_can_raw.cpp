#include "hi_can_raw.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <stdexcept>

using namespace hi_can;
using std::string;

RawCanInterface::RawCanInterface(const string& interfaceName)
    : _interfaceName(interfaceName),
      _socket(_createSocket,
              [this](int socket)
              { _configureSocket(socket); })
{
}

RawCanInterface::~RawCanInterface()
{
    // no need to close anything here, the FdWrapper classes will handle it
}

RawCanInterface::RawCanInterface(const RawCanInterface& other)
    : RawCanInterface(other._interfaceName)
{
    for (const auto& filter : other._filters)
        addFilter(filter);
}

RawCanInterface::RawCanInterface(RawCanInterface&& other) noexcept
{
    swap(*this, other);
}
RawCanInterface& RawCanInterface::operator=(RawCanInterface other)
{
    swap(*this, other);
    return *this;
}
RawCanInterface& RawCanInterface::operator=(RawCanInterface&& other) noexcept
{
    swap(*this, other);
    return *this;
}

void RawCanInterface::transmit(const Packet& packet)
{
    struct can_frame frame
    {
    };

    // set CAN ID, as well as necessary flags
    frame.can_id = packet.getAddress() | CAN_EFF_FLAG;  // use 29-bit extended frame format
    if (packet.getIsRTR())
        frame.can_id |= CAN_RTR_FLAG;

    frame.len = packet.getDataLen();
    if (frame.len > CAN_MAX_DLEN)
        throw std::runtime_error("Packet data length exceeds maximum CAN frame length");
    if (packet.getData().data() == nullptr)
        throw std::runtime_error("Packet data is null");  // note: this should never happen since we use std::array

    std::copy_n(packet.getData().begin(), frame.len, frame.data);

    const ssize_t bytesWritten = send((int)_socket, &frame, sizeof(frame), 0);
    if (bytesWritten < 0)
    {
        string err = std::strerror(errno);
        throw std::runtime_error("Failed to transmit CAN frame: " + err);
    }
    else if (bytesWritten != sizeof(frame))
        throw std::runtime_error("Failed to transmit entire CAN frame - only " +
                                 std::to_string(bytesWritten) +
                                 " bytes written of " +
                                 std::to_string(frame.len));
}

std::optional<Packet> hi_can::RawCanInterface::receive(bool blocking)
{
    struct can_frame frame
    {
    };
    const ssize_t bytesRead = recv((int)_socket, &frame, sizeof(frame), blocking ? 0 : MSG_DONTWAIT);
    if (bytesRead < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return std::nullopt;
        string err = std::strerror(errno);
        throw std::runtime_error("Failed to receive CAN frame: " + err);
    }
    if (bytesRead != sizeof(frame))
    {
        throw std::runtime_error("Failed to read entire CAN frame - only " +
                                 std::to_string(bytesRead) +
                                 " bytes read of " +
                                 std::to_string(sizeof(frame)));
    }

    const bool isRTR = (frame.can_id & CAN_RTR_FLAG) != 0;
    // note: should always be true since we only use extended frames
    const bool isEFF = (frame.can_id & CAN_EFF_FLAG) != 0;
    const address::raw_address_t address = frame.can_id & (isEFF ? CAN_EFF_MASK : CAN_SFF_MASK);
    Packet packet(address, frame.data, frame.len, isRTR);

    if (_receiveCallback)
        _receiveCallback(packet);

    return packet;
}

int RawCanInterface::_createSocket()
{
    // acquire a raw CAN socket
    int _socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (_socket < 0)
    {
        string err = std::strerror(errno);
        throw std::runtime_error("Failed to create CAN socket: " + err);
    }
    return _socket;
}
void RawCanInterface::_configureSocket(const int& socket)
{
    // Get the index of the network interface
    int interface_idx = 0;
    if (_interfaceName != "any")
    {
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, _interfaceName.c_str(), IFNAMSIZ - 1);
        if (ioctl(socket, SIOCGIFINDEX, &ifr) < 0)
        {
            string err = std::strerror(errno);
            throw std::runtime_error("Failed to get interface index: " + err);
        }
        interface_idx = ifr.ifr_ifindex;
    }

    // Bind socket to the specified CAN interface
    struct sockaddr_can addr
    {
    };
    addr.can_family = AF_CAN;
    addr.can_ifindex = interface_idx;
    if (bind(socket, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        string err = std::strerror(errno);
        throw std::runtime_error("Failed to bind CAN socket: " + err);
    }
}
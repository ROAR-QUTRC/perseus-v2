#include "hi_can_raw.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
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
    frame.can_id = static_cast<addressing::raw_address_t>(packet.getAddress());
    if (packet.getIsExtended())
        frame.can_id |= CAN_EFF_FLAG;
    if (packet.getIsRTR())
        frame.can_id |= CAN_RTR_FLAG;
    if (packet.getIsError())
        frame.can_id |= CAN_ERR_FLAG;

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

std::optional<Packet> RawCanInterface::receive(bool blocking)
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
    const bool isExtended = (frame.can_id & CAN_EFF_FLAG) != 0;
    const bool isError = (frame.can_id & CAN_ERR_FLAG) != 0;
    const addressing::flagged_address_t address(frame.can_id, isRTR, isError, isExtended);
    Packet packet(address, frame.data, frame.len);

    if (_receiveCallback)
        _receiveCallback(packet);

    return packet;
}

RawCanInterface& RawCanInterface::addFilter(const addressing::filter_t& address)
{
    FilteredCanInterface::addFilter(address);
    _updateFilters();
    return *this;
}
RawCanInterface& RawCanInterface::removeFilter(const addressing::filter_t& address)
{
    FilteredCanInterface::removeFilter(address);
    _updateFilters();
    return *this;
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
    if (socket < 0)
        throw std::runtime_error("Invalid socket file descriptor");
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

void RawCanInterface::_updateFilters()
{
    // build a vector containing the filters
    std::vector<can_filter> filters;
    if (!_filters.empty())
    {
        filters.reserve(_filters.size());
        for (const auto& filter : _filters)
        {
            canid_t can_id = static_cast<addressing::raw_address_t>(filter.address);
            if (filter.address.rtr)
                can_id |= CAN_RTR_FLAG;
            if (filter.address.error)
                can_id |= CAN_ERR_FLAG;
            if (filter.address.extended)
                can_id |= CAN_EFF_FLAG;

            canid_t can_mask = filter.mask | CAN_EFF_FLAG;
            if (filter.matchRtr)
                can_mask |= CAN_RTR_FLAG;
            if (filter.matchError)
                can_mask |= CAN_ERR_FLAG;
            filters.emplace_back(can_filter{.can_id = can_id, .can_mask = can_mask});
        }
    }
    else
    {
        // remove all filters (match everything)
        filters.emplace_back(can_filter{.can_id = 0, .can_mask = 0});
    }

    // apply the filters
    if (setsockopt((int)_socket, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), filters.size() * sizeof(can_filter)) < 0)
    {
        string err = std::strerror(errno);
        throw std::runtime_error("Failed to update CAN filters: " + err);
    }
}
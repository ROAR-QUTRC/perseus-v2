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

RawCanInterface::RawCanInterface(const string& interface_name)
    : _interface_name(interface_name),
      _socket(_create_socket,
              [this](int socket)
              { _configure_socket(socket); })
{
}

RawCanInterface::RawCanInterface(const RawCanInterface& other)
    : RawCanInterface(other._interface_name)
{
    for (const auto& filter : other._filters)
        add_filter(filter);
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

void RawCanInterface::transmit(const Packet& packet)
{
    struct can_frame frame{};

    // set CAN ID, as well as necessary flags
    frame.can_id = static_cast<addressing::raw_address_t>(packet.get_address());
    if (packet.get_is_extended())
        frame.can_id |= CAN_EFF_FLAG;
    if (packet.get_is_rtr())
        frame.can_id |= CAN_RTR_FLAG;
    if (packet.get_is_error())
        frame.can_id |= CAN_ERR_FLAG;

    frame.len = packet.get_data_len();
    if (frame.len > CAN_MAX_DLEN)
        throw std::runtime_error("Packet data length exceeds maximum CAN frame length");

    std::copy_n(packet.get_data().begin(), frame.len, frame.data);

    const ssize_t bytes_written = send((int)_socket, &frame, sizeof(frame), 0);
    if (bytes_written < 0)
    {
        string err = std::strerror(errno);
        throw std::runtime_error("Failed to transmit CAN frame: " + err);
    }
    else if (bytes_written != sizeof(frame))
        throw std::runtime_error("Failed to transmit entire CAN frame - only " +
                                 std::to_string(bytes_written) +
                                 " bytes written of " +
                                 std::to_string(frame.len));
}

std::optional<Packet> RawCanInterface::receive(bool blocking)
{
    struct can_frame frame{};
    const ssize_t bytes_read = recv((int)_socket, &frame, sizeof(frame), blocking ? 0 : MSG_DONTWAIT);
    if (bytes_read < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return std::nullopt;
        string err = std::strerror(errno);
        throw std::runtime_error("Failed to receive CAN frame: " + err);
    }
    if (bytes_read != sizeof(frame))
    {
        throw std::runtime_error("Failed to read entire CAN frame - only " +
                                 std::to_string(bytes_read) +
                                 " bytes read of " +
                                 std::to_string(sizeof(frame)));
    }

    const bool is_rtr = (frame.can_id & CAN_RTR_FLAG) != 0;
    // note: should always be true since we only use extended frames
    const bool is_extended = (frame.can_id & CAN_EFF_FLAG) != 0;
    const bool is_error = (frame.can_id & CAN_ERR_FLAG) != 0;
    const addressing::flagged_address_t address(frame.can_id, is_rtr, is_error, is_extended);
    Packet packet(address, frame.data, frame.len);

    if (_receive_callback)
        _receive_callback(packet);

    return packet;
}

RawCanInterface& RawCanInterface::add_filter(const addressing::filter_t& address)
{
    FilteredCanInterface::add_filter(address);
    _update_filters();
    return *this;
}
RawCanInterface& RawCanInterface::remove_filter(const addressing::filter_t& address)
{
    FilteredCanInterface::remove_filter(address);
    _update_filters();
    return *this;
}

int RawCanInterface::_create_socket()
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

void RawCanInterface::_configure_socket(const int& socket)
{
    if (socket < 0)
        throw std::runtime_error("Invalid socket file descriptor");
    // Get the index of the network interface
    int interface_idx = 0;
    if (_interface_name != "any")
    {
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, _interface_name.c_str(), IFNAMSIZ - 1);
        if (ioctl(socket, SIOCGIFINDEX, &ifr) < 0)
        {
            string err = std::strerror(errno);
            throw std::runtime_error("Failed to get interface index for \"" + _interface_name + "\": " + err);
        }
        interface_idx = ifr.ifr_ifindex;
    }

    // Bind socket to the specified CAN interface
    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = interface_idx;
    if (bind(socket, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        string err = std::strerror(errno);
        throw std::runtime_error("Failed to bind CAN socket on interface \"" + _interface_name + "\": " + err);
    }
}

void RawCanInterface::_update_filters()
{
    // build a vector containing the filters
    std::vector<can_filter> filters;
    if (!_filters.empty())
    {
        filters.reserve(_filters.size());
        for (const auto& filter : _filters)
        {
            canid_t can_id = static_cast<addressing::raw_address_t>(filter.address);
            if (filter.address.is_rtr)
                can_id |= CAN_RTR_FLAG;
            if (filter.address.is_error)
                can_id |= CAN_ERR_FLAG;
            if (filter.address.is_extended)
                can_id |= CAN_EFF_FLAG;

            canid_t can_mask = filter.mask | CAN_EFF_FLAG;
            if (filter.should_match_rtr)
                can_mask |= CAN_RTR_FLAG;
            if (filter.should_match_error)
                can_mask |= CAN_ERR_FLAG;
            filters.emplace_back(can_filter{.can_id = can_id, .can_mask = can_mask});
        }
    }
    else
    {
        // remove all filters (match everything)
        filters.emplace_back(can_filter{.can_id = 0, .can_mask = 0});
    }

    // TODO: Currently this appears to disable receiving *all* packets,
    // rather than applying the filters correctly, so disabled for now
    // apply the filters
    // if (setsockopt((int)_socket, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), filters.size() * sizeof(can_filter)) < 0)
    // {
    //     string err = std::strerror(errno);
    //     throw std::runtime_error("Failed to update CAN filters: " + err);
    // }
}
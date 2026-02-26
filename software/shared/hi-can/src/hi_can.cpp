#include "hi_can.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

using namespace hi_can;
using namespace hi_can::addressing;

void CanInterface::receive_all(bool block)
{
    receive(block);
    while (const auto packet = receive(false));
}

FilteredCanInterface& FilteredCanInterface::add_filter(const filter_t& filter)
{
    _filters.emplace(filter);
    return *this;
}
FilteredCanInterface& FilteredCanInterface::remove_filter(const filter_t& filter)
{
    _filters.erase(filter);
    return *this;
}

std::optional<filter_t> FilteredCanInterface::find_matching_filter(const flagged_address_t& address) const
{
    // because the filters are sorted by most specific mask first, even when multiple filters match,
    // it should generally match the most specific one first
    for (const auto& filter : _filters)
        if (filter.matches(address))
            return filter;
    return std::nullopt;
}
bool FilteredCanInterface::address_matches_filters(const flagged_address_t& address) const
{
    return _filters.empty() || find_matching_filter(address).has_value();
}

std::optional<Packet> SoftwareFilteredCanInterface::receive(bool blocking)
{
    const auto packet = _interface->receive(blocking);
    if (!packet || !address_matches_filters(packet->get_address()))
        return std::nullopt;

    if (_receive_callback)
        _receive_callback(*packet);
    return packet;
}
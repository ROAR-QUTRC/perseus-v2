#include "hi_can.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

using namespace hi_can;
using namespace hi_can::addressing;

using namespace std::chrono;

void CanInterface::receiveAll(bool block)
{
    receive(block);
    while (const auto packet = receive(false));
}

FilteredCanInterface& FilteredCanInterface::addFilter(const filter_t& filter)
{
    _filters.emplace(filter);
    return *this;
}
FilteredCanInterface& FilteredCanInterface::removeFilter(const filter_t& filter)
{
    _filters.erase(filter);
    return *this;
}

std::optional<filter_t> FilteredCanInterface::findMatchingFilter(const flagged_address_t& address) const
{
    // because the filters are sorted by most specific mask first, even when multiple filters match,
    // it should generally match the most specific one first
    for (const auto& filter : _filters)
        if (filter.matches(address))
            return filter;
    return std::nullopt;
}
bool FilteredCanInterface::addressMatchesFilters(const flagged_address_t& address) const
{
    return _filters.empty() || findMatchingFilter(address).has_value();
}

std::optional<Packet> SoftwareFilteredCanInterface::receive(bool blocking)
{
    const auto packet = _interface->receive(blocking);
    if (!packet || !addressMatchesFilters(packet->getAddress()))
        return std::nullopt;

    if (_receiveCallback)
        _receiveCallback(*packet);
    return packet;
}

PacketManager::PacketManager(FilteredCanInterface& interface)
    : _interface(interface)
{
    interface.setReceiveCallback([this](const Packet& packet)
                                 { this->_handleReceivedPacket(packet); });
}

void PacketManager::handleReceive()
{
    _interface.receiveAll(false);

    const auto now = steady_clock::now();
    for (auto& [key, value] : _callbacks)
    {
        if (!value.hasTimedOut && (now - value.lastReceived > value.config.timeout))
        {
            value.hasTimedOut = true;
            if (value.config.timeoutCallback)
                value.config.timeoutCallback();
        }
    }
}

void PacketManager::handleTransmit()
{
    const auto now = steady_clock::now();
    for (auto& [key, value] : _transmissions)
    {
        const auto elapsed = now - value.lastTransmitted;
        if ((value.config.interval > 0ms) && (elapsed > value.config.interval))
        {
            value.lastTransmitted = now;
            getInterface().transmit(value.config.packet);
        }
    }
}

void PacketManager::setCallback(const filter_t& filter, const callback_config_t& config)
{
    _callbacks[filter] = {
        .config = config,
        .hasTimedOut = false,
        .lastReceived = steady_clock::now(),
    };
    getInterface().addFilter(filter);
}

std::optional<PacketManager::callback_config_t> PacketManager::getCallback(const filter_t& filter)
{
    if (const auto it = _callbacks.find(filter); it != _callbacks.end())
        return it->second.config;
    return std::nullopt;
}

void PacketManager::removeCallback(const filter_t& filter)
{
    _callbacks.erase(filter);
}

void PacketManager::setTransmitConfig(const transmit_config_t& config)
{
    const bool hasZeroInterval = config.interval == 0ms;
    if (!hasZeroInterval)
        _transmissions[config.packet.getAddress()] = {
            .config = config,
            .lastTransmitted = steady_clock::now(),
        };
    else
        _transmissions.erase(config.packet.getAddress());

    if (hasZeroInterval || config.shouldTransmitImmediately)
        getInterface().transmit(config.packet);
}

void PacketManager::_handleReceivedPacket(const Packet& packet)
{
    // it should never happen that the packet address is not in the map since we're using a filtered interface,
    // but check anyway to not crash
    if (const std::optional<filter_t> filter = _interface.findMatchingFilter(packet.getAddress()); filter)
    {
        auto& callback = _callbacks[filter.value()];
        callback.lastPacket = packet;
        if (callback.config.dataCallback)
            callback.config.dataCallback(packet);

        if (callback.hasTimedOut)
        {
            callback.hasTimedOut = false;
            if (callback.config.timeoutRecoveryCallback)
                callback.config.timeoutRecoveryCallback(packet);
        }
        callback.lastReceived = steady_clock::now();
    }
}
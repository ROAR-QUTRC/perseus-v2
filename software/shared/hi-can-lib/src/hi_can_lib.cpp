#include "hi_can_lib.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

using namespace hi_can;

using namespace std::chrono;

void FilteredCanInterface::addAddressRange(const can_address_t& start, const can_address_t& end)
{
    for (can_address_t i = start; i <= end; i++)
        addAddress(i);
}
void FilteredCanInterface::removeAddressRange(const can_address_t& start, const can_address_t& end)
{
    for (can_address_t i = start; i <= end; i++)
        removeAddress(i);
}

std::optional<Packet> SoftwareFilteredCanInterface::receive()
{
    auto packet = _interface.receive();
    if (packet && _whitelist.find(packet->getAddress()) != _whitelist.end())
        return packet;
    return std::nullopt;
}

void SoftwareFilteredCanInterface::setReceiveCallback(const std::function<void(const Packet&)>& callback)
{
    _interface.setReceiveCallback([this, callback](const Packet& packet)
                                  {
                                     if (_whitelist.find(packet.getAddress()) != _whitelist.end())
                                         callback(packet); });
}

PacketManager::PacketManager(FilteredCanInterface& interface)
    : _interface(interface)
{
    interface.setReceiveCallback([this](const Packet& packet)
                                 { this->_handleReceivedPacket(packet); });
}

void PacketManager::handle()
{
    for (auto& [key, value] : _callbacks)
    {
        if (!value.hasTimedOut && (steady_clock::now() - value.lastReceived > value.config.timeout))
        {
            value.hasTimedOut = true;
            if (value.config.timeoutCallback)
                value.config.timeoutCallback();
        }
    }
    const auto now = steady_clock::now();
    for (auto& [key, value] : _transmissions)
    {
        const auto elapsed = now - value.lastTransmitted;
        if (elapsed > value.config.interval)
        {
            value.lastTransmitted = now;
            getInterface().transmit(value.config.packet);
        }
    }
}

void PacketManager::setCallback(const can_address_t& address, const callback_config_t& config)
{
    _callbacks[address] = {
        .config = config,
        .hasTimedOut = false,
        .lastReceived = steady_clock::now(),
    };
    _interface.addAddress(address);
}

void PacketManager::removeCallback(const can_address_t& address)
{
    _callbacks.erase(address);
    _interface.removeAddress(address);
}

void PacketManager::_handleReceivedPacket(const Packet& packet)
{
    // it should never happen that the packet address is not in the map since we're using a filtered interface,
    // but check anyway to not crash
    if (auto val = _callbacks.find(packet.getAddress()); val != _callbacks.end())
    {
        auto& callback = val->second;
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
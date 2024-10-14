#include "hi_can_lib.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

using namespace hi_can;

using namespace std::chrono;

PacketManager::PacketManager(CanInterface& interface)
    : _interface(interface)
{
    interface.setReceiveCallback([this](const Packet& packet)
                                 { this->_handlePacket(packet); });
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
}

void PacketManager::setCallback(const callback_config_t& config)
{
    _callbacks[config.address] = {
        .config = config,
        .hasTimedOut = false,
        .lastReceived = steady_clock::now(),
    };
}

void PacketManager::removeCallback(const can_address_t& address)
{
    _callbacks.erase(address);
}

void PacketManager::_handlePacket(const Packet& packet)
{
    if (_callbacks.find(packet.getAddress()) != _callbacks.end())
    {
        auto& callback = _callbacks[packet.getAddress()];
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
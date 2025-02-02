#include "hi_can_packet.hpp"

#include "hi_can.hpp"
#include "hi_can_parameter.hpp"

#define MISSED_PACKET_TIMEOUT_COUNT (3U)

using namespace hi_can;
using namespace hi_can::addressing;
using namespace hi_can::parameters;

using namespace std::chrono;

Packet::Packet(const flagged_address_t& address, const uint8_t data[], size_t dataLen)
{
    setAddress(address);
    setData(data, dataLen);
}
Packet::Packet(const flagged_address_t& address, const std::vector<uint8_t>& data)
{
    setAddress(address);
    setData(data);
}

void Packet::setData(const uint8_t data[], size_t dataLen)
{
    if (dataLen > MAX_PACKET_LEN)
        throw std::invalid_argument("Data is longer than the maximum packet length");

    if ((data == nullptr) && (dataLen > 0))
        throw std::invalid_argument("Data is nullptr but length is non-zero");

    _data.resize(dataLen);
    _data.assign(data, data + dataLen);
}
void Packet::setData(const std::vector<uint8_t>& data)
{
    if (data.size() > MAX_PACKET_LEN)
        throw std::invalid_argument("Data is longer than the maximum packet length");

    _data = data;
}

void Packet::setAddress(const flagged_address_t& address)
{
    if (address.address > MAX_ADDRESS)
        throw std::invalid_argument("Address is invalid");
    _address = address;
}

PacketManager::PacketManager(FilteredCanInterface& interface)
    : _interface(interface)
{
    interface.setReceiveCallback([this](const Packet& packet)
                                 { this->_handleReceivedPacket(packet); });
}

void PacketManager::handleReceive(bool shouldBlock)
{
    _interface.receiveAll(shouldBlock);

    const auto now = steady_clock::now();
    for (auto& [filter, config] : _callbacks)
    {
        if (!config.hasTimedOut && (now - config.lastReceived > (config.config.timeout * MISSED_PACKET_TIMEOUT_COUNT)))
        {
            config.hasTimedOut = true;
            if (config.config.timeoutCallback)
                config.config.timeoutCallback();
        }
    }
}

void PacketManager::handleTransmit(bool shouldForceTransmission)
{
    const auto now = steady_clock::now();
    for (auto& [address, config] : _transmissions)
    {
        const auto elapsed = now - config.lastTransmitted;
        if ((elapsed > config.config.interval) || shouldForceTransmission)
        {
            config.lastTransmitted = now;
            if (config.config.generator)
                getInterface().transmit(Packet(address, config.config.generator()));
            else
                getInterface().transmit(Packet(address));
        }
    }
}

void PacketManager::addGroup(const ParameterGroup& group)
{
    for (const auto& [filter, config] : group.getCallbacks())
        setCallback(filter, config);
    for (const auto& [address, config] : group.getTransmissions())
        setTransmissionConfig(address, config);
    for (const auto& transmission : group.getStartupTransmissions())
        getInterface().transmit(transmission);
}
void PacketManager::removeGroup(const ParameterGroup& group)
{
    for (const auto& [filter, config] : group.getCallbacks())
        removeCallback(filter);
    for (const auto& [address, config] : group.getTransmissions())
        removeTransmission(address);
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

void PacketManager::setTransmissionConfig(const flagged_address_t& address, const transmission_config_t& config)
{
    _transmissions[address] = {
        .config = config,
        .lastTransmitted = steady_clock::now(),
    };

    if (config.shouldTransmitImmediately)
    {
        if (config.generator)
            getInterface().transmit(Packet(address, config.generator()));
        else
            getInterface().transmit(Packet(address));
    }
}

void PacketManager::setTransmissionGenerator(const flagged_address_t& address, const data_generator_t& generator)
{
    if (const auto it = _transmissions.find(address); it != _transmissions.end())
        it->second.config.generator = generator;
}

void PacketManager::setTransmissionInterval(const flagged_address_t& address, const std::chrono::steady_clock::duration& interval)
{
    if (const auto it = _transmissions.find(address); it != _transmissions.end())
        it->second.config.interval = interval;
}

std::optional<PacketManager::transmission_config_t> PacketManager::getTransmissionConfig(const flagged_address_t& address)
{
    if (const auto it = _transmissions.find(address); it != _transmissions.end())
        return it->second.config;
    return std::nullopt;
}

void PacketManager::removeTransmission(const flagged_address_t& address)
{
    _transmissions.erase(address);
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
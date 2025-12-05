#include "hi_can_packet.hpp"

#include "hi_can.hpp"
#include "hi_can_parameter.hpp"

#define MISSED_PACKET_TIMEOUT_COUNT (3U)

using namespace hi_can;
using namespace hi_can::addressing;
using namespace hi_can::parameters;

using namespace std::chrono;

Packet::Packet(const flagged_address_t& address, const uint8_t data[], size_t data_len)
{
    set_address(address);
    set_data(data, data_len);
}
Packet::Packet(const flagged_address_t& address, const std::vector<uint8_t>& data)
{
    set_address(address);
    set_data(data);
}

void Packet::set_data(const uint8_t data[], size_t data_len)
{
    if (data_len > MAX_PACKET_LEN)
        throw std::invalid_argument("Data is longer than the maximum packet length");

    if ((data == nullptr) && (data_len > 0))
        throw std::invalid_argument("Data is nullptr but length is non-zero");

    _data.resize(data_len);
    _data.assign(data, data + data_len);
}
void Packet::set_data(const std::vector<uint8_t>& data)
{
    if (data.size() > MAX_PACKET_LEN)
        throw std::invalid_argument("Data is longer than the maximum packet length");

    _data = data;
}

void Packet::set_address(const flagged_address_t& address)
{
    if (address > MAX_ADDRESS)
        throw std::invalid_argument("Address is invalid");
    _address = address;
}

PacketManager::PacketManager(FilteredCanInterface& interface)
    : _interface(interface)
{
    interface.set_receive_callback([this](const Packet& packet)
                                   { this->_handle_received_packet(packet); });
}

void PacketManager::handle_receive(bool should_block)
{
    _interface.receive_all(should_block);

    const auto now = steady_clock::now();
    for (auto& [filter, config] : _callbacks)
    {
        if (!config.has_timed_out && (now - config.last_received > (config.config.timeout * MISSED_PACKET_TIMEOUT_COUNT)))
        {
            config.has_timed_out = true;
            if (config.config.timeout_callback)
                config.config.timeout_callback();
        }
    }
}

void PacketManager::handle_transmit(bool should_force_transmission)
{
    const auto now = steady_clock::now();
    for (auto& [address, config] : _transmissions)
    {
        const auto elapsed = now - config.last_transmitted;
        if ((elapsed > config.config.interval) || should_force_transmission)
        {
            config.last_transmitted = now;
            if (config.config.generator)
                get_interface().transmit(Packet(address, config.config.generator()));
            else
                get_interface().transmit(Packet(address));
        }
    }
}

void PacketManager::add_group(const ParameterGroup& group)
{
    for (const auto& [filter, config] : group.get_callbacks())
        set_callback(filter, config);
    for (const auto& [address, config] : group.get_transmissions())
        set_transmission_config(address, config);
    for (const auto& transmission : group.get_startup_transmissions())
        get_interface().transmit(transmission);
}
void PacketManager::remove_group(const ParameterGroup& group)
{
    for (const auto& [filter, config] : group.get_callbacks())
        remove_callback(filter);
    for (const auto& [address, config] : group.get_transmissions())
        remove_transmission(address);
}

void PacketManager::set_callback(const filter_t& filter, const callback_config_t& config)
{
    _callbacks[filter] = {
        .config = config,
        .has_timed_out = false,
        .last_received = steady_clock::now(),
    };
    get_interface().add_filter(filter);
}

std::optional<PacketManager::callback_config_t> PacketManager::get_callback(const filter_t& filter)
{
    if (const auto it = _callbacks.find(filter); it != _callbacks.end())
        return it->second.config;
    return std::nullopt;
}

void PacketManager::remove_callback(const filter_t& filter)
{
    _callbacks.erase(filter);
}

void PacketManager::set_transmission_config(const flagged_address_t& address, const transmission_config_t& config)
{
    _transmissions[address] = {
        .config = config,
        .last_transmitted = steady_clock::now(),
    };

    if (config.should_transmit_immediately)
    {
        if (config.generator)
            get_interface().transmit(Packet(address, config.generator()));
        else
            get_interface().transmit(Packet(address));
    }
}

void PacketManager::set_transmission_generator(const flagged_address_t& address, const data_generator_t& generator)
{
    if (const auto it = _transmissions.find(address); it != _transmissions.end())
        it->second.config.generator = generator;
}

void PacketManager::set_transmission_interval(const flagged_address_t& address, const std::chrono::steady_clock::duration& interval)
{
    if (const auto it = _transmissions.find(address); it != _transmissions.end())
        it->second.config.interval = interval;
}

std::optional<PacketManager::transmission_config_t> PacketManager::get_transmission_config(const flagged_address_t& address)
{
    if (const auto it = _transmissions.find(address); it != _transmissions.end())
        return it->second.config;
    return std::nullopt;
}

void PacketManager::remove_transmission(const flagged_address_t& address)
{
    _transmissions.erase(address);
}

void PacketManager::_handle_received_packet(const Packet& packet)
{
    // it should never happen that the packet address is not in the map since we're using a filtered interface,
    // but check anyway to not crash
    if (const std::optional<filter_t> filter = _interface.find_matching_filter(packet.get_address()); filter)
    {
        auto& callback = _callbacks[filter.value()];
        callback.last_packet = packet;
        if (callback.config.data_callback)
            callback.config.data_callback(packet);

        if (callback.has_timed_out)
        {
            callback.has_timed_out = false;
            if (callback.config.timeout_recovery_callback)
                callback.config.timeout_recovery_callback(packet);
        }
        callback.last_received = steady_clock::now();
    }
}
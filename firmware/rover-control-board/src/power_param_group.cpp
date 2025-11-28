#include "power_param_group.hpp"

#include <hi_can_twai.hpp>

#include "rover_core.hpp"

using namespace hi_can;
using namespace hi_can::addressing;

PowerBusParamGroup::PowerBusParamGroup(const standard_address_t& deviceAddress, power::distribution::rover_control_board::group bus)
    : _deviceAddress(deviceAddress)
{
    _callbacks.emplace_back(
        filter_t{
            .address = flagged_address_t(
                standard_address_t(deviceAddress,
                                   static_cast<uint8_t>(bus),
                                   static_cast<uint8_t>(addressing::power::distribution::rover_control_board::power_bus::parameter::CONTROL_IMMEDIATE)))},
        PacketManager::callback_config_t{
            .dataCallback = [this](const Packet& packet)
            {
                _immediateStatus.deserializeData(packet.getData());
            },
        });
    _callbacks.emplace_back(
        filter_t{
            .address = flagged_address_t(
                standard_address_t(deviceAddress,
                                   static_cast<uint8_t>(bus),
                                   static_cast<uint8_t>(addressing::power::distribution::rover_control_board::power_bus::parameter::CONTROL_SCHEDULED)))},
        PacketManager::callback_config_t{
            .dataCallback = [this](const Packet& packet)
            {
                _scheduledStatus.deserializeData(packet.getData());
                _scheduleReceived = coreGetUpTime();
            },
        });
    _callbacks.emplace_back(
        filter_t{
            .address = flagged_address_t(
                standard_address_t(deviceAddress,
                                   static_cast<uint8_t>(bus),
                                   static_cast<uint8_t>(addressing::power::distribution::rover_control_board::power_bus::parameter::CURRENT_LIMIT)))},
        PacketManager::callback_config_t{
            .dataCallback = [this](const Packet& packet)
            {
                _currentLimit.deserializeData(packet.getData());
            },
        });
}
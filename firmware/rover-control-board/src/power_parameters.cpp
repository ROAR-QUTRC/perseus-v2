#include "power_parameters.hpp"

#include <hi_can_twai.hpp>

#include "rover_core.hpp"

using namespace hi_can;
using namespace hi_can::addressing;

TwaiPowerBusParameterGroup::TwaiPowerBusParameterGroup(addressing::power::distribution::rover_control_board::group bus,
                                                       std::function<void(bool)> busStateHandler,
                                                       std::function<void(void)> clearErrorHandler)
    : _busId(bus)
{
    // Immediate control
    _callbacks.emplace_back(
        filter_t{
            .address = flagged_address_t(
                standard_address_t(_deviceAddress,
                                   static_cast<uint8_t>(bus),
                                   static_cast<uint8_t>(addressing::power::distribution::rover_control_board::power_bus::parameter::CONTROL_IMMEDIATE)))},
        PacketManager::callback_config_t{
            .data_callback = [&](const Packet& packet)
            {
                _immediateStatus.deserialize_data(packet.get_data());
                if (_immediateStatus.clear_error && clearErrorHandler)
                {
                    clearErrorHandler();
                }
                busStateHandler(_immediateStatus.bus_target_state);
            },
        });

    // Scheduled control
    _callbacks.emplace_back(
        filter_t{
            .address = flagged_address_t(
                standard_address_t(_deviceAddress,
                                   static_cast<uint8_t>(bus),
                                   static_cast<uint8_t>(addressing::power::distribution::rover_control_board::power_bus::parameter::CONTROL_SCHEDULED)))},
        PacketManager::callback_config_t{
            .data_callback = [this](const Packet& packet)
            {
                _scheduledStatus.deserialize_data(packet.get_data());
                _busOffTimer = _scheduledStatus.bus_off_time;
                _busOnTimer = _scheduledStatus.bus_on_time;
            },
        });

    // Current limit
    _callbacks.emplace_back(
        filter_t{
            .address = flagged_address_t(
                standard_address_t(_deviceAddress,
                                   static_cast<uint8_t>(bus),
                                   static_cast<uint8_t>(addressing::power::distribution::rover_control_board::power_bus::parameter::CURRENT_LIMIT)))},
        PacketManager::callback_config_t{
            .data_callback = [this](const Packet& packet)
            {
                memcpy(&_currentLimit, &(packet.get_data()), sizeof(uint32_t));
            },
        });
    _busStateHandler = busStateHandler;
    // Timer for scheduled control
    _oneSecondTimer = timerCreate(_oneSecondTimerCallback,
                                  1000, true, this);
}

void TwaiPowerBusParameterGroup::setBusStatus(parameters::power::distribution::power_status status)
{
    _currentStatus.status = status;
}

parameters::power::distribution::power_status TwaiPowerBusParameterGroup::getBusStatus()
{
    return _currentStatus.status;
}

hi_can::parameters::power::distribution::status_t TwaiPowerBusParameterGroup::getStatus()
{
    return _currentStatus;
}

addressing::power::distribution::rover_control_board::group TwaiPowerBusParameterGroup::getId()
{
    return _busId;
}

void TwaiPowerBusParameterGroup::setBusCurrent(uint32_t current)
{
    _currentStatus.current = current;
}

uint32_t TwaiPowerBusParameterGroup::getBusCurrent()
{
    return _currentStatus.current;
}

void TwaiPowerBusParameterGroup::setBusVoltage(uint16_t voltage)
{
    _currentStatus.voltage = voltage;
}

uint16_t TwaiPowerBusParameterGroup::getBusVoltage()
{
    return _currentStatus.voltage;
}

uint32_t TwaiPowerBusParameterGroup::getLimitCurrent()
{
    return _currentLimit;
}

void TwaiPowerBusParameterGroup::_busSetOn(bool state)
{
    if (_busStateHandler)
        _busStateHandler(state);
}

void TwaiPowerBusParameterGroup::_oneSecondTimerCallback(TimerHandle_t timer)
{
    TwaiPowerBusParameterGroup* source = (TwaiPowerBusParameterGroup*)pvTimerGetTimerID(timer);

    if (source->_busOffTimer)
    {
        source->_busOffTimer--;
        if (source->_busOffTimer == 0)
            source->_busSetOn(false);
    }

    if (source->_busOnTimer)
    {
        source->_busOnTimer--;
        if (source->_busOnTimer == 0)
            source->_busSetOn(true);
    }
}
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
            .dataCallback = [&](const Packet& packet)
            {
                _immediateStatus.deserializeData(packet.getData());
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
            .dataCallback = [this](const Packet& packet)
            {
                _scheduledStatus.deserializeData(packet.getData());
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
            .dataCallback = [this](const Packet& packet)
            {
                memcpy(&_currentLimit, &(packet.getData()), sizeof(uint32_t));
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

parameters::power::distribution::power_status TwaiPowerBusParameterGroup::getBusStatus(void)
{
    return _currentStatus.status;
}

hi_can::parameters::power::distribution::status_t TwaiPowerBusParameterGroup::getStatus(void)
{
    return _currentStatus;
}

addressing::power::distribution::rover_control_board::group TwaiPowerBusParameterGroup::getId(void)
{
    return _busId;
}

void TwaiPowerBusParameterGroup::setBusCurrent(uint32_t current)
{
    _currentStatus.current = current;
}

uint32_t TwaiPowerBusParameterGroup::getBusCurrent(void)
{
    return _currentStatus.current;
}

void TwaiPowerBusParameterGroup::setBusVoltage(uint16_t voltage)
{
    _currentStatus.voltage = voltage;
}

uint16_t TwaiPowerBusParameterGroup::getBusVoltage(void)
{
    return _currentStatus.voltage;
}

uint32_t TwaiPowerBusParameterGroup::getLimitCurrent(void)
{
    return static_cast<uint32_t>(_currentLimit);
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
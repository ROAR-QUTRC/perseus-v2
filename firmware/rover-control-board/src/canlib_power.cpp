#include "canlib_power.hpp"

// core libs
#include <rover_thread.hpp>

// app libs
#include "canlib_power.hpp"
#include "rcb.hpp"
#include "sdkconfig.h"

using std::function;

bool CanlibPowerBusParameterGroup::statusIsError(canlib_power_status status)
{
    return !(status == CANLIB_POWER_OFF ||
             status == CANLIB_POWER_ON ||
             status == CANLIB_POWER_PRECHARGING);
}

CanlibPowerBusParameterGroup::CanlibPowerBusParameterGroup(uint8_t paramGroup,
                                                           function<void(bool)> busStateHandler,
                                                           function<void(void)> clearErrorHandler,
                                                           function<void(uint32_t)> currentLimitHandler)
    : _busId(paramGroup)
{
    // to pass member functions for use as standard function pointers,
    // we must bind them with their "first" parameter being the `this` pointer
    // and then allow a placeholder so it can be used as intended
    addParameter("status",
                 canlib_parameter_description{
                     .address = {
                         .group     = paramGroup,
                         .parameter = CANLIB_PARAM_POWER_STATUS,
                     },
                     .writable  = false,
                     .broadcast = true,
                     .interval  = 100},
                 canlib_power_status_data{});
    addParameter("immediate_control",
                 canlib_parameter_description{
                     .address = {
                         .group     = paramGroup,
                         .parameter = CANLIB_PARAM_POWER_CONTROL_IMMEDIATE},
                     .writable            = true,
                     .data_change_handler = [&](auto addr)
                     { _immediateDataCallback(addr); }},
                 canlib_power_control_immediate_data{});
    addParameter("scheduled_control",
                 canlib_parameter_description{
                     .address = {
                         .group     = paramGroup,
                         .parameter = CANLIB_PARAM_POWER_CONTROL_SCHEDULED},
                     .writable            = true,
                     .data_change_handler = [&](auto addr)
                     { _scheduledDataCallback(addr); }},
                 canlib_power_control_scheduled_data{});
    addParameter<uint32_t>("limit",
                           canlib_parameter_description{
                               .address =
                                   {
                                       .group     = paramGroup,
                                       .parameter = CANLIB_PARAM_CURRENT_LIMIT,
                                   },
                               .writable            = true,
                               .data_change_handler = [&](auto addr)
                               { _limitDataCallback(addr); }},
                           CONFIG_DEFAULT_SOFTWARE_FUSE_CURRENT);

    setBusStateHandler(busStateHandler);
    setClearErrorHandler(clearErrorHandler);
    setCurrentLimitHandler(currentLimitHandler);
    _oneSecondTimer = timerCreate(_oneSecondTimerCallback,
                                  1000, true, this);
}
CanlibPowerBusParameterGroup::~CanlibPowerBusParameterGroup()
{
    xTimerDelete(_oneSecondTimer, 0);
}

void CanlibPowerBusParameterGroup::setBusStateHandler(function<void(bool)> busStateHandler)
{
    _busStateHandler = busStateHandler;
}
void CanlibPowerBusParameterGroup::setClearErrorHandler(function<void(void)> clearErrorHandler)
{
    _clearErrorHandler = clearErrorHandler;
}
void CanlibPowerBusParameterGroup::setCurrentLimitHandler(std::function<void(uint32_t)> currentLimitHandler)
{
    _currentLimitHandler = currentLimitHandler;
}

void CanlibPowerBusParameterGroup::setVoltage(uint16_t voltage)
{
    _status.voltage = voltage;
    _updateStatus();
}
void CanlibPowerBusParameterGroup::setCurrent(uint32_t current)
{
    _status.current = current;
    _updateStatus();
}

void CanlibPowerBusParameterGroup::setBusStatus(canlib_power_status status)
{
    _status.status = status;
    _updateStatus();
}
bool CanlibPowerBusParameterGroup::getRequestedState()
{
    canlib_power_control_immediate_data data;
    getParameter("immediate_control", data);
    return data.bus_target_state;
}

uint32_t CanlibPowerBusParameterGroup::getLimitCurrent()
{
    uint32_t data = CONFIG_DEFAULT_SOFTWARE_FUSE_CURRENT;
    getParameter("limit", data);
    return data;
}

uint8_t CanlibPowerBusParameterGroup::getId()
{
    return _busId;
}

void CanlibPowerBusParameterGroup::_oneSecondTimerCallback(TimerHandle_t timer)
{
    CanlibPowerBusParameterGroup* source = (CanlibPowerBusParameterGroup*)pvTimerGetTimerID(timer);

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

void CanlibPowerBusParameterGroup::_immediateDataCallback(canlib_address address)
{
    canlib_power_control_immediate_data data;
    if (canlibGetParameter(address, data) != CANLIB_OK)
        return;

    if (data.clear_error && _clearErrorHandler)
        _clearErrorHandler();
    _busSetOn(data.bus_target_state);
}
void CanlibPowerBusParameterGroup::_scheduledDataCallback(canlib_address address)
{
    canlib_power_control_scheduled_data data;
    if (canlibGetParameter(address, data) != CANLIB_OK)
        return;

    if (_busOffTimer && !data.bus_off_time)
    {
        CANLIB_DEBUG("Bus %d shutdown request cancelled!", _busId);
    }
    _busOffTimer = data.bus_off_time;
    if (_busOffTimer)
    {
        CANLIB_DEBUG("Bus %d shutdown requested in %ds", _busId, _busOffTimer);
    }

    if (_busOnTimer && !data.bus_on_time)
    {
        CANLIB_DEBUG("Bus %d startup request cancelled!", _busId);
    }
    _busOnTimer = data.bus_on_time;
    if (_busOnTimer)
    {
        CANLIB_DEBUG("Bus %d startup requested in %ds", _busId, _busOnTimer);
    }

    if (data.bus_off_time || data.bus_on_time)
        xTimerReset(_oneSecondTimer, 0);
}
void CanlibPowerBusParameterGroup::_limitDataCallback(canlib_address address)
{
    uint32_t data = CONFIG_DEFAULT_SOFTWARE_FUSE_CURRENT;
    if (canlibGetParameter(address, data) != CANLIB_OK)
        return;

    // limit s/w fuse current to sensible values
    if (data > RCB_MAX_CURRENT)
    {
        data = RCB_MAX_CURRENT;
        canlibSetParameter(address, data);
    }

    CANLIB_DEBUG("Current limit now %lumA", data);
    if (_currentLimitHandler)
        _currentLimitHandler(data);
}
void CanlibPowerBusParameterGroup::_busSetOn(bool state)
{
    if (state != _prevState)
    {
        _prevState = state;
        CANLIB_INFO("Bus %d turning %s", _busId, state ? "on" : "off");
    }
    if (_busStateHandler)
        _busStateHandler(state);
}

void CanlibPowerBusParameterGroup::_updateStatus()
{
    setParameter("status", _status);
}
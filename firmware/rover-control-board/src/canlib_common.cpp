#include "canlib_common.hpp"

// rover libs
#include <rover_core.hpp>
#include <rover_thread.hpp>
#include <canlib.hpp>

CanlibCommonParameterGroup::CanlibCommonParameterGroup(canlib_handler_t shutdownHandler,
                                                             canlib_handler_t rebootHandler)
{
    addParameter("status",
                 canlib_parameter_description{
                     .address   = {.group = CANLIB_GROUP_COMMON, .parameter = CANLIB_PARAM_COMMON_STATUS},
                     .writable  = false,
                     .broadcast = true,
                     .interval  = 1000},
                 canlib_common_status_data{});
    addParameter("power",
                 canlib_parameter_description{
                     .address             = {.group = CANLIB_GROUP_COMMON, .parameter = CANLIB_PARAM_COMMON_POWER},
                     .writable            = true,
                     .data_change_handler = [&](auto address)
                     { _powerDataCallback(address); }},
                 canlib_common_power_data{});
    addParameter("cpu",
                 canlib_parameter_description{
                     .address   = {.group = CANLIB_GROUP_COMMON, .parameter = CANLIB_PARAM_COMMON_CPU},
                     .writable  = false,
                     .broadcast = true,
                     .interval  = 1000},
                 canlib_common_cpu_data{});

    setShutdownHandler(shutdownHandler);
    setRebootHandler(rebootHandler);
    _oneSecondTimer = timerCreate(_oneSecondTimerCallback,
                                  1000, true, this);
}

CanlibCommonParameterGroup::~CanlibCommonParameterGroup()
{
    xTimerDelete(_oneSecondTimer, 0);
}

void CanlibCommonParameterGroup::setShutdownHandler(canlib_handler_t handler)
{
    _shutdownHandler = handler;
}
void CanlibCommonParameterGroup::setRebootHandler(canlib_handler_t handler)
{
    _rebootHandler = handler;
}

void CanlibCommonParameterGroup::setStatus(canlib_common_status state, int16_t error)
{
    _status.state      = state;
    _status.error_code = error;
}

void CanlibCommonParameterGroup::_oneSecondTimerCallback(TimerHandle_t timer)
{
    CanlibCommonParameterGroup* source = (CanlibCommonParameterGroup*)pvTimerGetTimerID(timer);
    if (canlib_error err =
            source->setParameter("cpu", canlib_common_cpu_data{
                                            .uptime    = (uint32_t)coreGetUptime(),
                                            .cpu_usage = coreGetUsage()});
        err != CANLIB_OK)
        CANLIB_WARN("Error setting CPU data: %d", (int)err);

    if (source->_shutdownTimer)
    {
        source->_shutdownTimer--;
        CANLIB_WARN("Shutting down from CAN bus request");
        if ((source->_shutdownTimer == 0) && source->_shutdownHandler)
        {
            source->_shutdownHandler();
        }
    }

    if (source->_rebootTimer)
    {
        source->_rebootTimer--;
        CANLIB_WARN("Rebooting from CAN bus request");
        if ((source->_rebootTimer == 0) && source->_rebootHandler)
        {
            source->_rebootHandler();
        }
    }
}

void CanlibCommonParameterGroup::_powerDataCallback(canlib_address address)
{
    canlib_common_power_data data;
    if (canlibGetParameter(address, data) != CANLIB_OK)
        return;

    if (_shutdownTimer && !data.shutdown_time)
        CANLIB_WARN("Shutdown request cancelled!");
    _shutdownTimer = data.shutdown_time;
    if (_shutdownTimer)
        CANLIB_WARN("Shutdown requested in %ds", _shutdownTimer);

    if (_rebootTimer && !data.reboot_time)
        CANLIB_WARN("Reboot request cancelled!");
    _rebootTimer = data.reboot_time;
    if (_rebootTimer)
        CANLIB_WARN("Reboot requested in %ds", _rebootTimer);

    if (data.immediate_shutdown)
    {
        CANLIB_WARN("Immediate shutdown requested!");
        if (_shutdownHandler)
            _shutdownHandler();
    }
    if (data.immediate_reboot)
    {
        CANLIB_WARN("Immediate reboot requested!");
        if (_rebootHandler)
            _rebootHandler();
    }

    if (data.shutdown_time || data.reboot_time)
        xTimerReset(_oneSecondTimer, 0);
}
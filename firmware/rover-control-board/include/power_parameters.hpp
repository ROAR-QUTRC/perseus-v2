#pragma once

#include "rover_thread.hpp"

class TwaiPowerBusParameterGroup : public hi_can::parameters::ParameterGroup
{
public:
    TwaiPowerBusParameterGroup(hi_can::addressing::power::distribution::rover_control_board::group bus,
                               std::function<void(bool)> busStateHandler,
                               std::function<void(void)> clearErrorHandler);
    void setBusStatus(hi_can::parameters::power::distribution::power_status status);
    hi_can::parameters::power::distribution::power_status getBusStatus(void);
    hi_can::parameters::power::distribution::status_t getStatus(void);
    hi_can::addressing::power::distribution::rover_control_board::group getId(void);
    uint16_t getBusVoltage(void);
    void setBusVoltage(uint16_t);
    uint32_t getBusCurrent(void);
    void setBusCurrent(uint32_t);
    uint32_t getLimitCurrent(void);

private:
    hi_can::parameters::power::distribution::immediate_control_t _immediateStatus;
    hi_can::parameters::power::distribution::scheduled_control_t _scheduledStatus;
    uint32_t _currentLimit;

    uint8_t _busOffTimer = 0;
    uint8_t _busOnTimer = 0;
    hi_can::addressing::power::distribution::rover_control_board::group _busId;
    const hi_can::addressing::standard_address_t _deviceAddress = hi_can::addressing::standard_address_t(
        hi_can::addressing::power::SYSTEM_ID,
        hi_can::addressing::power::distribution::SUBSYSTEM_ID,
        hi_can::addressing::power::distribution::rover_control_board::DEVICE_ID);

    hi_can::parameters::power::distribution::status_t _currentStatus;
    TimerHandle_t _oneSecondTimer;
    static void _oneSecondTimerCallback(TimerHandle_t timer);
    std::function<void(bool)> _busStateHandler;
    void _busSetOn(bool on);
};
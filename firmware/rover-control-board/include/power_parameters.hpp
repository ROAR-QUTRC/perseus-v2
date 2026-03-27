#pragma once

#include <hi_can.hpp>

#include "rover_thread.hpp"
#include "sdkconfig.h"

constexpr hi_can::addressing::standard_address_t RCB_DEVICE_ADDRESS{
    hi_can::addressing::power::SYSTEM_ID,
    hi_can::addressing::power::distribution::SUBSYSTEM_ID,
    hi_can::addressing::power::distribution::rover_control_board::DEVICE_ID,
};

class TwaiPowerBusParameterGroup : public hi_can::parameters::ParameterGroup
{
public:
    TwaiPowerBusParameterGroup(hi_can::addressing::power::distribution::rover_control_board::group bus,
                               std::function<void(bool)> busStateHandler,
                               std::function<void(void)> clear_errorHandler);
    void set_bus_status(hi_can::parameters::power::distribution::power_status status);
    hi_can::parameters::power::distribution::power_status get_bus_status(void);
    hi_can::parameters::power::distribution::status_t get_status(void);
    hi_can::addressing::power::distribution::rover_control_board::group get_id(void);
    uint16_t get_bus_voltage(void);
    void set_bus_voltage(uint16_t);
    uint32_t get_bus_current(void);
    void set_bus_current(uint32_t);
    uint32_t get_limit_current(void);

private:
    hi_can::parameters::power::distribution::immediate_control_t _immediateStatus;
    hi_can::parameters::power::distribution::scheduled_control_t _scheduledStatus;
    uint32_t _currentLimit = CONFIG_DEFAULT_SOFTWARE_FUSE_CURRENT;

    uint8_t _busOffTimer = 0;
    uint8_t _busOnTimer = 0;
    hi_can::addressing::power::distribution::rover_control_board::group _bus_id;
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

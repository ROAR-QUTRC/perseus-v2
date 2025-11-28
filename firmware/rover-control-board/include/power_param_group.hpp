#pragma once

#include "rcb.hpp"

class PowerBusParamGroup : public hi_can::parameters::ParameterGroup
{
public:
    PowerBusParamGroup(const hi_can::addressing::standard_address_t& deviceAddress, hi_can::addressing::power::distribution::rover_control_board::group bus);
    void setBusStatus(parameters::power::distribution::power_status status);
    uint16_t getBusVoltage(void);
    uint32_t getBusCurrent(void);

private:
    hi_can::parameters::power::distribution::immediate_control_t _immediateStatus;
    hi_can::parameters::power::distribution::scheduled_control_t _scheduledStatus;
    int64_t _scheduleReceived = 0;
    hi_can::parameters::SimpleSerializable<uint32_t> _currentLimit = 0;

    hi_can::addressing::standard_address_t _deviceAddress;
};
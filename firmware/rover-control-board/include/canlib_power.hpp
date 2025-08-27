#pragma once

// standard libs
#include <cstdint>
#include <functional>

// rover libs
#include <canlib.hpp>

// core libs
#include <driver/gpio.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_oneshot.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

enum canlib_power_status : uint8_t
{
    CANLIB_POWER_OFF,            // bus off
    CANLIB_POWER_ON,             // bus on
    CANLIB_POWER_PRECHARGING,    // bus is precharging
    CANLIB_POWER_SHORT_CIRCUIT,  // precharging failed - short circuit
    CANLIB_POWER_SWITCH_FAILED,  // main switch not turning on - estop?
    CANLIB_POWER_OVERLOAD,       // software fuse triggered
    CANLIB_POWER_FAULT,          // switch reporting fault
};

enum canlib_power_subsystem : uint8_t
{
    CANLIB_SUBSYSTEM_POWER_CTRL = 0x00,
};

enum canlib_power_control_devices : uint8_t
{
    CANLIB_DEVICE_ROVER_CONTROL_BOARD = 0x00,
};

enum canlib_power_control_param_groups : uint8_t
{
    CANLIB_GROUP_POWER_CONTACTOR = 0x01,
    CANLIB_GROUP_POWER_COMPUTE_BUS = 0x02,
    CANLIB_GROUP_POWER_DRIVE_BUS = 0x03,
    CANLIB_GROUP_POWER_AUX_BUS = 0x04,
    CANLIB_GROUP_POWER_SPARE_BUS = 0x05,
};

enum canlib_power_control_contactor_parameters : uint8_t
{
    CANLIB_PARAM_POWER_CONTACTOR = 0x00,
};

enum canlib_power_control_bus_parameters : uint8_t
{
    CANLIB_PARAM_POWER_CONTROL_IMMEDIATE = 0x00,
    CANLIB_PARAM_POWER_CONTROL_SCHEDULED = 0x01,
    CANLIB_PARAM_CURRENT_LIMIT = 0x02,
    CANLIB_PARAM_POWER_STATUS = 0x03,
};

#pragma pack(1)
struct canlib_power_contactor_data
{
    bool immediate_shutdown : 1 = false;
    uint8_t _reserved : 7 = 0;   // padding to make a full byte
    uint8_t shutdown_timer = 0;  // if a non-0 value is received, shutdown in that many seconds
};

#pragma pack(1)
struct canlib_power_status_data
{
    canlib_power_status status = CANLIB_POWER_OFF;
    uint16_t voltage = 0;  // in mV
    uint32_t current = 0;  // in mA
};

#pragma pack(1)
struct canlib_power_control_immediate_data
{
    bool bus_target_state : 1 = false;  // bus on/off state
    bool clear_error : 1 = false;       // retry if an error has occurred
    uint8_t _reserved : 6 = 0;          // padding to make a full byte
};

#pragma pack(1)
struct canlib_power_control_scheduled_data
{
    uint8_t bus_off_time = 0;  // if a non-0 value is received, turn off bus in that many seconds
    uint8_t bus_on_time = 0;   // if a non-0 value is received, turn on bus in that many seconds
};

class CanlibPowerBusParameterGroup : public CanlibParameterGroup
{
public:
    static bool statusIsError(canlib_power_status status);

    CanlibPowerBusParameterGroup(uint8_t paramGroup,
                                 std::function<void(bool)> busStateHandler = CANLIB_NO_HANDLER,
                                 std::function<void(void)> clearErrorHandler = CANLIB_NO_HANDLER,
                                 std::function<void(uint32_t)> currentLimitHandler = CANLIB_NO_HANDLER);
    virtual ~CanlibPowerBusParameterGroup();

    void setBusStateHandler(std::function<void(bool)> busStateHandler);
    void setClearErrorHandler(std::function<void(void)> clearErrorHandler);
    void setCurrentLimitHandler(std::function<void(uint32_t)> currentLimitHandler);

    void setVoltage(uint16_t voltage);
    void setCurrent(uint32_t current);

    void setBusStatus(canlib_power_status status);
    bool getRequestedState();
    uint32_t getLimitCurrent();
    uint8_t getId();

private:
    static void _oneSecondTimerCallback(TimerHandle_t timer);

    void _immediateDataCallback(canlib_address address);
    void _scheduledDataCallback(canlib_address address);
    void _limitDataCallback(canlib_address address);

    void _busSetOn(bool state);

    void _updateStatus();

    canlib_power_status_data _status;

    const uint8_t _busId;
    bool _prevState = false;
    std::function<void(bool)> _busStateHandler;
    std::function<void(void)> _clearErrorHandler;
    std::function<void(uint32_t)> _currentLimitHandler;

    uint8_t _busOffTimer = 0;
    uint8_t _busOnTimer = 0;

    TimerHandle_t _oneSecondTimer;  // handles bus startup + shutdown timing
};
#pragma once

// standard libs
#include <cstdint>

// core
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

// CANlib
#include "canlib_types.hpp"

enum canlib_common_params : uint8_t
{
    CANLIB_PARAM_COMMON_POWER = 0x00,   // power state control (eg remote reboot)
    CANLIB_PARAM_COMMON_STATUS = 0x01,  // status
    CANLIB_PARAM_COMMON_CPU = 0x02,     // uptime + CPU loading
};

enum canlib_common_status : uint8_t
{
    CANLIB_STATE_NORMAL,
    CANLIB_STATE_STARTING,
    CANLIB_STATE_SHUTTING_DOWN,
    CANLIB_STATE_REBOOTING,
    CANLIB_STATE_WARNING,
    CANLIB_STATE_ERROR,
    CANLIB_STATE_FATAL_ERROR,
};

#pragma pack(1)
struct canlib_common_status_data
{
    uint8_t state = CANLIB_STATE_STARTING;
    int16_t error_code = 0;
};

#pragma pack(1)
struct canlib_common_power_data
{
    bool immediate_shutdown : 1 = false;  // set true to request immediate shutdown
    bool immediate_reboot : 1 = false;    // set true to request immediate reboot
    uint8_t _reserved : 6 = 0;            // padding to make a full byte

    uint8_t shutdown_time = 0;  // if a non-0 value is received, shutdown in that many seconds
    uint8_t reboot_time = 0;    // if a non-0 value is received, reboot in that many seconds
};

#pragma pack(1)
struct canlib_common_cpu_data
{
    uint32_t uptime = 0;      // in millis
    uint8_t cpu_usage = 100;  // in %
};

class CanlibCommonParameterGroup : public CanlibParameterGroup
{
public:
    CanlibCommonParameterGroup(std::function<void(void)> shutdownHandler = CANLIB_NO_HANDLER,
                               std::function<void(void)> rebootHandler = CANLIB_NO_HANDLER);
    virtual ~CanlibCommonParameterGroup();

    void setShutdownHandler(canlib_handler_t handler);
    void setRebootHandler(canlib_handler_t handler);

    void setStatus(canlib_common_status state, int16_t error = CANLIB_NO_ERROR);

private:
    static void _oneSecondTimerCallback(TimerHandle_t timer);
    void _powerDataCallback(canlib_address address);

    canlib_common_status_data _status = {};

    std::function<void(void)> _shutdownHandler = CANLIB_NO_HANDLER;
    std::function<void(void)> _rebootHandler = CANLIB_NO_HANDLER;
    uint8_t _shutdownTimer = 0;
    uint8_t _rebootTimer = 0;

    TimerHandle_t _oneSecondTimer;  // handles shutdown + reboot timing
};
#pragma once

// core
#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <freertos/FreeRTOS.h>

// standard libraries
#include <atomic>
#include <cstdint>

// rover libs - old canbus
/*
#include <canlib.hpp>
#include <canlib_power.hpp>
*/
// new canbus
#include <hi_can_twai.hpp>

// PRECHARGING MATH
// the timer value is the time to precharge for, in tenths of a ms - 100 = 10ms
// energy dump into the resistors in the event of a short is given by (t*V^2)/(2R)
// which, for 25mS is 1.69J

// if measuring above this voltage, switch is *definitely* outputting an error level
#define RCB_BUS_CUR_SENSE_ERROR_VTG 3000
// see dkILIS in switch datasheet - outputs 1mA on sense pin for 50000mA flowing through
#define RCB_BUS_CUR_SENSE_FACTOR 50000UL
// 1k resistor to gnd from switch sense pin
#define RCB_BUS_CUR_SENSE_RESISTOR 1000

// disable switch error between these voltages (capacitors discharge slowly below 5V, and switch error turns off below 2V)
#define RCB_SWITCH_ERR_DISABLE_MIN_VTG 2000
#define RCB_SWITCH_ERR_DISABLE_MAX_VTG 5000
#define RCB_BUS_ON_VOLTAGE             16000
#define RCB_MAX_CURRENT                100000UL  // max 100A per channel

// 100k-10k voltage divider to measure bus vtg
#define RCB_ADC_TO_BUS_VTG(_vtg) ROVER_ADC_DIVIDER_TO_SOURCE_VTG(_vtg, 100, 10)
// convert current feedback voltage to bus current
#define RCB_ADC_TO_BUS_CURRENT(_vtg) (((_vtg) * RCB_BUS_CUR_SENSE_FACTOR) / RCB_BUS_CUR_SENSE_RESISTOR)

class RoverPowerBus
{
public:
    enum class bus_state
    {
        OFF = 0,
        PRECHARGING,
        ON,
        PRECHARGE_COOLDOWN,
        ERROR,
    };
    enum class bus_error
    {
        NONE,
        PRECHARGE_FAIL,
        SWITCH_FAIL,
        SWITCH_ERROR,
        OVERLOAD,
    };
    RoverPowerBus(uint8_t busId, uint16_t prechargeVoltage,
                  gpio_num_t precharge, gpio_num_t mainSwitch,
                  gpio_num_t vtgFeedback, gpio_num_t currentFeedback);
    ~RoverPowerBus();
    void setBusOn(bool on);
    void clearError();
    void handle();

    bool isBusOn() { return ((_state != bus_state::OFF) || (_state != bus_state::ERROR)); }

private:
    const static gptimer_alarm_config_t _prechargeOffConfig;
    const static gptimer_alarm_config_t _prechargeOnConfig;
    static bool _timerCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx);

    const gpio_num_t _prechargePin;
    const gpio_num_t _switchPin;
    const gpio_num_t _vtgFeedback;
    const gpio_num_t _currentFeedback;

    const uint32_t _prechargeVtg;

    gptimer_handle_t _timer = NULL;
    bool _switchHadError = false;
    int64_t _switchOnTime = 0;
    std::atomic<int64_t> _switchOffTime = 0;
    int32_t _switchErrorCounter = 0;

    std::atomic<bus_state> _state = bus_state::OFF;  // force a state change
    // we can't switch states in an ISR, so queue the change and handle it in the main loop
    std::atomic<bus_state> _nextState = bus_state::OFF;
    std::atomic<uint8_t> _retryCount = 0;
    std::atomic<bus_error> _errorCode = bus_error::NONE;
};

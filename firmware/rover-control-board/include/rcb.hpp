#pragma once

// core
#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <freertos/FreeRTOS.h>

// standard libraries
#include <atomic>
#include <cstdint>

// new canbus
#include <hi_can_twai.hpp>

#include "power_parameters.hpp"

// PRECHARGING MATH
// the timer value is the time to precharge for, in tenths of a ms - 100 = 10ms
// energy dump into the resistors in the event of a short is given by (t*V^2)/(2R)
// which, for 25mS is 1.69J

// SPARE
// R17 = 1.323k R16 = 54.066k R18 = 2.087k
const unsigned long SPARE_R16 = 54066;
const unsigned int SPARE_R17 = 1323;
const unsigned int SPARE_R19 = 0;

// DRIVE
// R17 = 1.312k R16 = 53.997k R18 = 2.053k
const unsigned long DRIVE_R16 = 53997;
const unsigned int DRIVE_R17 = 1312;
const unsigned int DRIVE_R19 = 0;

// COMPUTE
// R17 = 1.377k R16 = 54.065k R18 = 2.094k
const unsigned long COMPUTE_R16 = 54065;
const unsigned int COMPUTE_R17 = 1377;
const unsigned int COMPUTE_R19 = 0;

// AUX
// R17 = 1.339k R16 = 54.026k R18 = 2.070k
const unsigned long AUX_R16 = 54026;
const unsigned int AUX_R17 = 1339;
const unsigned int AUX_R19 = 0;

// if measuring above this voltage, switch is *definitely* outputting an error level
const unsigned int RCB_BUS_CURRENT_SENSE_ERROR_VOLTAGE = 3000;
// see dkILIS in switch datasheet - outputs 1mA on sense pin for 50000mA flowing through
const unsigned long RCB_BUS_CURRENT_SENSE_FACTOR = 50000UL;
// 1k resistor to gnd from switch sense pin
const unsigned int RCB_BUS_CURRENT_SENSE_RESISTOR = 1000;

// disable switch error between these voltages (capacitors discharge slowly below 5V, and switch error turns off below 2V)
const unsigned int RCB_SWITCH_ERROR_DISABLE_MIN_VOLTAGE = 2000;
const unsigned int RCB_SWITCH_ERROR_DISABLE_MAX_VOLTAGE = 5000;
const unsigned long RCB_BUS_ON_VOLTAGE = 16000;
const unsigned long RCB_MAX_CURRENT = 50000UL;  // max 50A per channel

// 100k-10k voltage divider to measure bus voltage
// static long rcb_adc_to_bus_voltage(const long voltage)
// {
//     return ((voltage * (100 + 10)) / 10);
// }
// // convert current feedback voltage to bus current
// static long rcb_adc_to_bus_current(const long voltage)
// {
//     return (((voltage)*RCB_BUS_CURRENT_SENSE_FACTOR) / RCB_BUS_CURRENT_SENSE_RESISTOR);
// }

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
    RoverPowerBus(hi_can::addressing::power::distribution::rover_control_board::group bus_id, uint16_t precharge_voltage,
                  gpio_num_t precharge, gpio_num_t main_switch,
                  gpio_num_t voltage_feedback, gpio_num_t current_feedback, const int R16, const long R17, const int R19);
    ~RoverPowerBus();
    void set_bus_state(bool on);
    void clear_error();
    void handle();
    hi_can::PacketManager::transmission_config_t get_transmission_config(void);
    TwaiPowerBusParameterGroup get_parameter_group(void);

    bool is_bus_on() { return ((_state != bus_state::OFF) && (_state != bus_state::ERROR)); }
    unsigned long adc_to_bus_voltage(const unsigned long voltage);
    unsigned long adc_to_bus_current(const unsigned long voltage);

private:
    const static gptimer_alarm_config_t _precharge_off_config;
    const static gptimer_alarm_config_t _precharge_on_config;
    static bool _timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx);

    TwaiPowerBusParameterGroup _can_parameters;
    hi_can::PacketManager::transmission_config_t _status_transmission_config;

    const gpio_num_t _precharge_pin;
    const gpio_num_t _switch_pin;
    const gpio_num_t _voltage_feedback;
    const gpio_num_t _current_feedback;

    const uint32_t _precharge_voltage;

    unsigned long _R16;
    unsigned int _R17;
    unsigned int _R19;

    gptimer_handle_t _timer = NULL;
    bool _switch_had_error = false;
    int64_t _switch_on_time = 0;
    std::atomic<int64_t> _switch_off_time = 0;
    int32_t _switch_error_counter = 0;

    std::atomic<bus_state> _state = bus_state::OFF;  // force a state change
    // we can't switch states in an ISR, so queue the change and handle it in the main loop
    std::atomic<bus_state> _next_state = bus_state::OFF;
    std::atomic<uint8_t> _retry_count = 0;
    std::atomic<bus_error> _error_code = bus_error::NONE;
};

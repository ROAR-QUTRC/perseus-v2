#include "rcb.hpp"

#include <rover_adc.hpp>
#include <rover_core.hpp>
#include <rover_io.hpp>
#include <rover_log.hpp>
#include <rover_thread.hpp>
using namespace hi_can::parameters::power::distribution;

const gptimer_alarm_config_t RoverPowerBus::_precharge_off_config = {
    .alarm_count = CONFIG_PRECHARGE_TIME * 10,
    .reload_count = 0,
    .flags = {
        .auto_reload_on_alarm = false,
    },
};
const gptimer_alarm_config_t RoverPowerBus::_precharge_on_config = {
    .alarm_count = CONFIG_PRECHARGE_RETRY_WAIT_TIME * 10,
    .reload_count = 0,
    .flags = {
        .auto_reload_on_alarm = false,
    },
};

RoverPowerBus::RoverPowerBus(hi_can::addressing::power::distribution::rover_control_board::group bus_id, uint16_t precharge_voltage,
                             gpio_num_t precharge, gpio_num_t main_switch,
                             gpio_num_t voltage_feedback, gpio_num_t current_feedback)
    : _can_parameters(bus_id, [&](bool on)
                      { set_bus_state(on); }, [&]()
                      { clear_error(); }),
      _precharge_pin(precharge),
      _switch_pin(main_switch),
      _voltage_feedback(voltage_feedback),
      _current_feedback(current_feedback),
      _precharge_voltage(precharge_voltage)
{
    gpio_set_output(precharge);
    gpio_set_output(main_switch);
    adc_set_channel_enabled(voltage_feedback, true, true);
    adc_set_channel_enabled(current_feedback, true, false);
    adc_set_error_voltage(current_feedback, RCB_BUS_CURRENT_SENSE_ERROR_VOLTAGE);

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 10000,
        .intr_priority = 0,
        .flags = {
            .intr_shared = true,
        },
    };

    gptimer_new_timer(&timer_config, &_timer);
    gptimer_event_callbacks_t timer_callbacks = {
        .on_alarm = _timer_callback,
    };
    gptimer_register_event_callbacks(_timer, &timer_callbacks, this);
    gptimer_set_alarm_action(_timer, &_precharge_off_config);
    gptimer_enable(_timer);

    hi_can::PacketManager::transmission_config_t _status_transmission_config = {
        .generator = [&](void)
        {
            return _can_parameters.get_status().serialize_data();
        },
        .interval = std::chrono::nanoseconds(100000000)};  // 100 milliseconds

    _can_parameters.set_bus_status(power_status::OFF);
    _can_parameters.set_bus_voltage(0);
    _can_parameters.set_bus_current(0);
}

RoverPowerBus::~RoverPowerBus()
{
    set_bus_state(false);
    gpio_reset_pin(_precharge_pin);
    gpio_reset_pin(_switch_pin);
    adc_set_channel_enabled(_voltage_feedback, false, true);
    adc_set_channel_enabled(_current_feedback, false, false);
    gpio_reset_pin(_voltage_feedback);
    gpio_reset_pin(_current_feedback);
    gptimer_disable(_timer);
    gptimer_del_timer(_timer);
}

void RoverPowerBus::set_bus_state(bool on)
{
    if (on)
    {
        if (_state == bus_state::OFF)
            _next_state = bus_state::PRECHARGING;
    }
    else if (_state != bus_state::ERROR)
        _next_state = bus_state::OFF;
}

void RoverPowerBus::clear_error()
{
    if (_state == bus_state::ERROR)
    {
        printf("Bus %d resetting error state!", _can_parameters.get_id());
        _next_state = bus_state::OFF;
        handle();
    }
    if (_retry_count >= CONFIG_PRECHARGE_RETRY_COUNT)
    {
        printf("Bus %d resetting precharge retry counter - was %d", _can_parameters.get_id(), _retry_count.load());
        _retry_count = 0;
    }
}

void RoverPowerBus::handle()
{
    const int32_t bus_voltage = rcb_adc_to_bus_voltage(adc_get_voltage(_voltage_feedback));
    _can_parameters.set_bus_voltage(bus_voltage);
    const int32_t current_sense_voltage = adc_get_voltage(_current_feedback);
    const uint32_t bus_current = rcb_adc_to_bus_current(current_sense_voltage);

    // check software fusing
    const bool switch_error = (current_sense_voltage == ROVER_ADC_ERR_RETURN_VAL);
    const bool disable_switch_error = ((bus_voltage > RCB_SWITCH_ERROR_DISABLE_MIN_VOLTAGE) && (bus_voltage < RCB_SWITCH_ERROR_DISABLE_MAX_VOLTAGE));

    const int64_t now = core_get_uptime();
    const int64_t bus_on_period = now - _switch_on_time;
    const int64_t bus_off_period = now - _switch_off_time;

    const bool bus_overloaded = (bus_current > _can_parameters.get_limit_current());
    const bool capacitors_discharging = (bus_off_period < CONFIG_BUS_CAP_DISCHARGE_TIME);
    const bool inrush_done = (bus_on_period > CONFIG_PRECHARGE_INRUSH_TIME);
    const bool bus_should_be_on = (_state == bus_state::ON) || (_next_state == bus_state::ON);
    const bool bus_is_on = (bus_voltage > RCB_BUS_ON_VOLTAGE);

    if (switch_error)
    {
        _switch_had_error = true;
        if (!capacitors_discharging && (_state != bus_state::PRECHARGING) && !disable_switch_error)
            _switch_error_counter++;
        if ((_switch_error_counter > CONFIG_SWITCH_ERROR_DEBOUNCE_TIME) && (_state != bus_state::ERROR))
        {
            printf("Bus %d switch error!", _can_parameters.get_id());
            _switch_error_counter = CONFIG_SWITCH_ERROR_DEBOUNCE_TIME;
            _can_parameters.set_bus_current(0);
            _next_state = bus_state::ERROR;
            _error_code = bus_error::SWITCH_ERROR;
        }
    }
    else
    {
        if (_switch_error_counter)
            _switch_error_counter--;
        _can_parameters.set_bus_current(bus_current);

        if (_switch_had_error)
            _switch_had_error = false;  // allow one cycle of delay for average to move
        else if (bus_overloaded && inrush_done && bus_should_be_on)
        {
            if (_state != bus_state::ERROR)
            {
                printf("Bus %d overload: %lumA/%lumA", _can_parameters.get_id(), bus_current, _can_parameters.get_limit_current());
                _next_state = bus_state::ERROR;
                _error_code = bus_error::OVERLOAD;
            }
        }
    }
    if (bus_should_be_on && !bus_is_on && inrush_done)
    {
        if (_state != bus_state::ERROR)
        {
            printf("Bus %d switch fail: Bus voltage %ldmV/%ldmV", _can_parameters.get_id(), bus_voltage, RCB_BUS_ON_VOLTAGE);
            _next_state = bus_state::ERROR;
            _error_code = bus_error::SWITCH_FAIL;
        }
    }

    if (_next_state == _state)
        return;  // only process if state has changed

    _state = _next_state.load();
    switch (_state)
    {
    default:
    case bus_state::OFF:
        printf("Bus %d now OFF", _can_parameters.get_id());
        _can_parameters.set_bus_status(power_status::OFF);
        _switch_off_time = core_get_uptime();
        _retry_count = 0;

        gptimer_stop(_timer);
        gpio_set_level(_precharge_pin, 0);
        gpio_set_level(_switch_pin, 0);
        break;
    case bus_state::PRECHARGING:
        printf("Bus %d precharging", _can_parameters.get_id());
        _can_parameters.set_bus_status(power_status::PRECHARGING);

        // set gptimer timeout
        gptimer_set_alarm_action(_timer, &_precharge_off_config);

        gpio_set_level(_precharge_pin, 1);
        gptimer_set_raw_count(_timer, 0);
        gptimer_start(_timer);
        _switch_on_time = core_get_uptime();
        break;
    case bus_state::ON:
        printf("Bus %d now ON", _can_parameters.get_id());
        _retry_count = 0;
        _can_parameters.set_bus_status(power_status::ON);
        break;
    case bus_state::PRECHARGE_COOLDOWN:
        printf("Bus %d precharge failed (%05ldmV/%05lumV) - waiting %ldms, retry #%d", _can_parameters.get_id(),
               bus_voltage, _precharge_voltage, CONFIG_PRECHARGE_RETRY_WAIT_TIME, _retry_count.load());
        _can_parameters.set_bus_status(power_status::PRECHARGING);

        // set gptimer timeout
        gptimer_set_alarm_action(_timer, &_precharge_on_config);

        gpio_set_level(_precharge_pin, 0);
        gpio_set_level(_switch_pin, 0);
        gptimer_set_raw_count(_timer, 0);
        gptimer_start(_timer);
        break;
    case bus_state::ERROR:
        printf("Bus %d ERROR! Error code: %d", _can_parameters.get_id(), (int)_error_code.load());

        if (_error_code == bus_error::PRECHARGE_FAIL)
            _can_parameters.set_bus_status(power_status::SHORT_CIRCUIT);
        else if (_error_code == bus_error::SWITCH_FAIL)
            _can_parameters.set_bus_status(power_status::SWITCH_FAILED);
        else if (_error_code == bus_error::OVERLOAD)
            _can_parameters.set_bus_status(power_status::OVERLOAD);
        else
            _can_parameters.set_bus_status(power_status::FAULT);

        gptimer_stop(_timer);
        gpio_set_level(_precharge_pin, 0);
        gpio_set_level(_switch_pin, 0);
        break;
    }
}

bool RoverPowerBus::_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx)
{
    RoverPowerBus* source = (RoverPowerBus*)user_ctx;
    gpio_set_level(source->_precharge_pin, 0);
    gptimer_stop(timer);

    if (source->_state == bus_state::PRECHARGING)
    {
        const uint32_t voltage = rcb_adc_to_bus_voltage(adc_get_voltage(source->_voltage_feedback));
        if (voltage > source->_precharge_voltage)
        {
            gpio_set_level(source->_switch_pin, 1);
            source->_next_state = bus_state::ON;
        }
        else
        {
            source->_retry_count++;
            source->_switch_off_time = core_get_uptime();
            if (source->_retry_count < CONFIG_PRECHARGE_RETRY_COUNT)
                source->_next_state = bus_state::PRECHARGE_COOLDOWN;
            else
            {
                source->_next_state = bus_state::ERROR;
                source->_error_code = bus_error::PRECHARGE_FAIL;
            }
        }
    }
    else  // precharge cooldown (completed)
    {
        // retry precharging
        source->_next_state = bus_state::PRECHARGING;
    }
    return true;
}

hi_can::PacketManager::transmission_config_t RoverPowerBus::get_transmission_config(void)
{
    return _status_transmission_config;
}

TwaiPowerBusParameterGroup RoverPowerBus::get_parameter_group(void)
{
    return _can_parameters;
}
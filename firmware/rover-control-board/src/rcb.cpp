#include "rcb.hpp"

#include <rover_adc.hpp>
#include <rover_core.hpp>
#include <rover_io.hpp>
#include <rover_log.hpp>
#include <rover_thread.hpp>
using namespace hi_can::parameters::power::distribution;

const gptimer_alarm_config_t RoverPowerBus::_prechargeOffConfig = {
    .alarm_count = CONFIG_PRECHARGE_TIME * 10,
    .reload_count = 0,
    .flags = {
        .auto_reload_on_alarm = false,
    },
};
const gptimer_alarm_config_t RoverPowerBus::_prechargeOnConfig = {
    .alarm_count = CONFIG_PRECHARGE_RETRY_WAIT_TIME * 10,
    .reload_count = 0,
    .flags = {
        .auto_reload_on_alarm = false,
    },
};

RoverPowerBus::RoverPowerBus(hi_can::addressing::power::distribution::rover_control_board::group busId, uint16_t prechargeVoltage,
                             gpio_num_t precharge, gpio_num_t mainSwitch,
                             gpio_num_t voltageFeedback, gpio_num_t currentFeedback)
    : _canParameters(busId, [&](bool on)
                     { setBusOn(on); }, [&]()
                     { clearError(); }),
      _prechargePin(precharge),
      _switchPin(mainSwitch),
      _voltageFeedback(voltageFeedback),
      _currentFeedback(currentFeedback),
      _prechargeVoltage(prechargeVoltage)
{
    ioConfigOutput(precharge);
    ioConfigOutput(mainSwitch);
    adcSetChannelEnabled(voltageFeedback, true, true);
    adcSetChannelEnabled(currentFeedback, true, false);
    adcSetErrorVoltage(currentFeedback, RCB_BUS_CURRENT_SENSE_ERROR_VOLTAGE);

    gptimer_config_t timerConfig = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 10000,
        .intr_priority = 0,
        .flags = {
            .intr_shared = true,
        },
    };

    gptimer_new_timer(&timerConfig, &_timer);
    gptimer_event_callbacks_t timerCallbacks = {
        .on_alarm = _timerCallback,
    };
    gptimer_register_event_callbacks(_timer, &timerCallbacks, this);
    gptimer_set_alarm_action(_timer, &_prechargeOffConfig);
    gptimer_enable(_timer);

    hi_can::PacketManager::transmission_config_t _statusTransmissionConfig = {
        .generator = [&](void)
        {
            return _canParameters.getStatus().serializeData();
        },
        .interval = std::chrono::nanoseconds(100000000)};  // 100 milli seconds

    _canParameters.setBusStatus(power_status::OFF);
    _canParameters.setBusVoltage(0);
    _canParameters.setBusCurrent(0);
}

RoverPowerBus::~RoverPowerBus()
{
    setBusOn(false);
    gpio_reset_pin(_prechargePin);
    gpio_reset_pin(_switchPin);
    adcSetChannelEnabled(_voltageFeedback, false, true);
    adcSetChannelEnabled(_currentFeedback, false, false);
    gpio_reset_pin(_voltageFeedback);
    gpio_reset_pin(_currentFeedback);
    gptimer_disable(_timer);
    gptimer_del_timer(_timer);
}

void RoverPowerBus::setBusOn(bool on)
{
    if (on)
    {
        if (_state == bus_state::OFF)
            _nextState = bus_state::PRECHARGING;
    }
    else if (_state != bus_state::ERROR)
        _nextState = bus_state::OFF;
}

void RoverPowerBus::clearError()
{
    if (_state == bus_state::ERROR)
    {
        WARN("Bus %d resetting error state!", _canParameters.getId());
        _nextState = bus_state::OFF;
        handle();
    }
    if (_retryCount >= CONFIG_PRECHARGE_RETRY_COUNT)
    {
        WARN("Bus %d resetting precharge retry counter - was %d", _canParameters.getId(), _retryCount.load());
        _retryCount = 0;
    }
}

void RoverPowerBus::handle()
{
    const int32_t bus_voltage = RCB_ADC_TO_BUS_VOLTAGE(adcGetVoltage(_voltageFeedback));
    _canParameters.setBusVoltage(bus_voltage);
    const int32_t curSenseVtg = adcGetVoltage(_currentFeedback);
    const uint32_t busCurrent = RCB_ADC_TO_BUS_CURRENT(curSenseVtg);

    // check software fusing
    const bool switchErr = (curSenseVtg == ROVER_ADC_ERR_RETURN_VAL);
    const bool disableSwitchErr = ((bus_voltage > RCB_SWITCH_ERROR_DISABLE_MIN_VOLTAGE) && (bus_voltage < RCB_SWITCH_ERROR_DISABLE_MAX_VOLTAGE));
    // WARN("%d %d %05ld, %05ld", switchErr, disableSwitchErr, curSenseVtg, RCB_ADC_TO_BUS_VOLTAGE(adcGetVoltage(_voltageFeedback)));

    const int64_t now = coreGetUptime();
    const int64_t busOnPeriod = now - _switchOnTime;
    const int64_t busOffPeriod = now - _switchOffTime;

    const bool busOverloaded = (busCurrent > _canParameters.getLimitCurrent());
    const bool capacitorsDischarging = (busOffPeriod < CONFIG_BUS_CAP_DISCHARGE_TIME);
    const bool inrushDone = (busOnPeriod > CONFIG_PRECHARGE_INRUSH_TIME);
    const bool busShouldBeOn = (_state == bus_state::ON) || (_nextState == bus_state::ON);
    const bool busIsOn = (bus_voltage > RCB_BUS_ON_VOLTAGE);
    // WARN("%d %d %05ld", busShouldBeOn, busIsOn, bus_voltage);
    if (switchErr)
    {
        _switchHadError = true;
        if (!capacitorsDischarging && (_state != bus_state::PRECHARGING) && !disableSwitchErr)
            _switchErrorCounter++;
        if ((_switchErrorCounter > CONFIG_SWITCH_ERROR_DEBOUNCE_TIME) && (_state != bus_state::ERROR))
        {
            ERROR("Bus %d switch error!", _canParameters.getId());
            _switchErrorCounter = CONFIG_SWITCH_ERROR_DEBOUNCE_TIME;
            _canParameters.setBusCurrent(0);
            _nextState = bus_state::ERROR;
            _errorCode = bus_error::SWITCH_ERROR;
        }
    }
    else
    {
        if (_switchErrorCounter)
            _switchErrorCounter--;
        _canParameters.setBusCurrent(busCurrent);

        if (_switchHadError)
            _switchHadError = false;  // allow one cycle of delay for average to move
        else if (busOverloaded && inrushDone && busShouldBeOn)
        {
            if (_state != bus_state::ERROR)
            {
                ERROR("Bus %d overload: %lumA/%lumA", _canParameters.getId(), busCurrent, _canParameters.getLimitCurrent());
                _nextState = bus_state::ERROR;
                _errorCode = bus_error::OVERLOAD;
            }
        }
    }
    if (busShouldBeOn && !busIsOn && inrushDone)
    {
        if (_state != bus_state::ERROR)
        {
            ERROR("Bus %d switch fail: Bus voltage %ldmV/%ldmV", _canParameters.getId(), bus_voltage, RCB_BUS_ON_VOLTAGE);
            _nextState = bus_state::ERROR;
            _errorCode = bus_error::SWITCH_FAIL;
        }
    }

    if (_nextState == _state)
        return;  // only process if state has changed

    _state = _nextState.load();
    switch (_state)
    {
    default:
    case bus_state::OFF:
        INFO("Bus %d now OFF", _canParameters.getId());
        _canParameters.setBusStatus(power_status::OFF);
        _switchOffTime = coreGetUptime();
        _retryCount = 0;

        gptimer_stop(_timer);
        gpio_set_level(_prechargePin, 0);
        gpio_set_level(_switchPin, 0);
        break;
    case bus_state::PRECHARGING:
        DEBUG("Bus %d precharging", _canParameters.getId());
        _canParameters.setBusStatus(power_status::PRECHARGING);

        // set gptimer timeout
        gptimer_set_alarm_action(_timer, &_prechargeOffConfig);

        gpio_set_level(_prechargePin, 1);
        gptimer_set_raw_count(_timer, 0);
        gptimer_start(_timer);
        _switchOnTime = coreGetUptime();
        break;
    case bus_state::ON:
        INFO("Bus %d now ON", _canParameters.getId());
        _retryCount = 0;
        _canParameters.setBusStatus(power_status::ON);
        break;
    case bus_state::PRECHARGE_COOLDOWN:
        WARN("Bus %d precharge failed (%05ldmV/%05lumV) - waiting %ldms, retry #%d", _canParameters.getId(),
             bus_voltage, _prechargeVoltage, CONFIG_PRECHARGE_RETRY_WAIT_TIME, _retryCount.load());
        _canParameters.setBusStatus(power_status::PRECHARGING);

        // set gptimer timeout
        gptimer_set_alarm_action(_timer, &_prechargeOnConfig);

        gpio_set_level(_prechargePin, 0);
        gpio_set_level(_switchPin, 0);
        gptimer_set_raw_count(_timer, 0);
        gptimer_start(_timer);
        break;
    case bus_state::ERROR:
        ERROR("Bus %d ERROR! Error code: %d", _canParameters.getId(), (int)_errorCode.load());

        if (_errorCode == bus_error::PRECHARGE_FAIL)
            _canParameters.setBusStatus(power_status::SHORT_CIRCUIT);
        else if (_errorCode == bus_error::SWITCH_FAIL)
            _canParameters.setBusStatus(power_status::SWITCH_FAILED);
        else if (_errorCode == bus_error::OVERLOAD)
            _canParameters.setBusStatus(power_status::OVERLOAD);
        else
            _canParameters.setBusStatus(power_status::FAULT);

        gptimer_stop(_timer);
        gpio_set_level(_prechargePin, 0);
        gpio_set_level(_switchPin, 0);
        break;
    }
}

bool RoverPowerBus::_timerCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx)
{
    RoverPowerBus* source = (RoverPowerBus*)user_ctx;
    gpio_set_level(source->_prechargePin, 0);
    gptimer_stop(timer);

    if (source->_state == bus_state::PRECHARGING)
    {
        const uint32_t vtg = RCB_ADC_TO_BUS_VOLTAGE(adcGetVoltage(source->_voltageFeedback));
        if (vtg > source->_prechargeVoltage)
        {
            gpio_set_level(source->_switchPin, 1);
            source->_nextState = bus_state::ON;
        }
        else
        {
            source->_retryCount++;
            source->_switchOffTime = coreGetUptime();
            if (source->_retryCount < CONFIG_PRECHARGE_RETRY_COUNT)
                source->_nextState = bus_state::PRECHARGE_COOLDOWN;
            else
            {
                source->_nextState = bus_state::ERROR;
                source->_errorCode = bus_error::PRECHARGE_FAIL;
            }
        }
    }
    else  // precharge cooldown (completed)
    {
        // retry precharging
        source->_nextState = bus_state::PRECHARGING;
    }
    return true;
}

hi_can::PacketManager::transmission_config_t RoverPowerBus::GetTransmissionConfig(void)
{
    return _statusTransmissionConfig;
}

TwaiPowerBusParameterGroup RoverPowerBus::GetParameterGroup(void)
{
    return _canParameters;
}
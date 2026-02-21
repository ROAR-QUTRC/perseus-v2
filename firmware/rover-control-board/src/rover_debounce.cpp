#include "rover_debounce.hpp"

#include <rover_log.hpp>

IoDebouncedButton::IoDebouncedButton(gpio_num_t pin, gpio_pull_mode_t pullMode, bool activeLevel)
    : _activeLevel(activeLevel),
      _pin(pin)
{
    _last_pressTime = 0;
    gpio_set_input(pin, pullMode);
}

void IoDebouncedButton::handle()
{
    bool pressed = (gpio_get_level(_pin) == _activeLevel);
    const uint64_t now = core_get_uptime();

    if (pressed)
    {
        if (_last_pressTime == 0)
        {
            // WARN("First press");
            _last_pressTime = now;
        }
        else if ((now - _last_pressTime) < DEBOUNCE_PRESS_TIME)  // capture if not past debounce time
            ;
        else if (!_isPressed)
        {
            if ((now - _lastReleaseTime) < DEBOUNCE_MULTI_PRESS_TIME)
            {
                _repeatPressCount++;
            }
            _isPressed = true;
        }
        else if ((now - _last_pressTime) >= DEBOUNCE_HOLD_TIME)
        {
            if (!_isHeld)
            {
                _has_hold = true;
            }
            _isHeld = true;
        }
    }
    else
    {
        if (_last_pressTime && ((now - _last_pressTime) >= DEBOUNCE_PRESS_TIME))
        {
            _last_pressTime = 0;
            _lastReleaseTime = now;
            if (!_isHeld && _isPressed)
            {
                _has_press = true;
            }
            _isPressed = false;
            _isHeld = false;
            clear_has_hold();
        }
        else if (_repeatPressCount && (now - _lastReleaseTime) >= DEBOUNCE_MULTI_PRESS_TIME)
        {
            _repeatPressCount = 0;
        }
        _last_pressTime = 0;
    }
}

bool IoDebouncedButton::has_press()
{
    if (_has_press)
    {
        clear_has_press();
        return true;
    }
    return false;
}
bool IoDebouncedButton::has_hold()
{
    if (_has_hold)
    {
        clear_has_hold();
        return true;
    }
    return false;
}
bool IoDebouncedButton::isPressed()
{
    return _isPressed;
}
bool IoDebouncedButton::isHeld()
{
    return _isHeld;
}
int IoDebouncedButton::get_repeat_press_count()
{
    return _repeatPressCount;
}

void IoDebouncedButton::clear_has_press()
{
    _has_press = false;
}
void IoDebouncedButton::clear_has_hold()
{
    _has_hold = false;
}
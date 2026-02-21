#include "rover_debounce.hpp"

IoDebouncedButton::IoDebouncedButton(gpio_num_t pin, gpio_pull_mode_t pullMode, bool activeLevel)
    : _activeLevel(activeLevel),
      _pin(pin)
{
    _last_press_time = 0;
    gpio_set_input(pin, pullMode);
}

void IoDebouncedButton::handle()
{
    bool pressed = (gpio_get_level(_pin) == _activeLevel);
    const uint64_t now = core_get_uptime();

    if (pressed)
    {
        if (_last_press_time == 0)
        {
            _last_press_time = now;
        }
        else if ((now - _last_press_time) < DEBOUNCE_PRESS_TIME)  // capture if not past debounce time
            ;
        else if (!_is_pressed)
        {
            if ((now - _last_release_time) < DEBOUNCE_MULTI_PRESS_TIME)
            {
                _repeat_press_count++;
            }
            _is_pressed = true;
        }
        else if ((now - _last_press_time) >= DEBOUNCE_HOLD_TIME)
        {
            if (!_is_held)
            {
                _has_hold = true;
            }
            _is_held = true;
        }
    }
    else
    {
        if (_last_press_time && ((now - _last_press_time) >= DEBOUNCE_PRESS_TIME))
        {
            _last_press_time = 0;
            _last_release_time = now;
            if (!_is_held && _is_pressed)
            {
                _has_press = true;
            }
            _is_pressed = false;
            _is_held = false;
            clear_has_hold();
        }
        else if (_repeat_press_count && (now - _last_release_time) >= DEBOUNCE_MULTI_PRESS_TIME)
        {
            _repeat_press_count = 0;
        }
        _last_press_time = 0;
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
bool IoDebouncedButton::is_pressed()
{
    return _is_pressed;
}
bool IoDebouncedButton::is_held()
{
    return _is_held;
}
int IoDebouncedButton::get_repeat_press_count()
{
    return _repeat_press_count;
}

void IoDebouncedButton::clear_has_press()
{
    _has_press = false;
}
void IoDebouncedButton::clear_has_hold()
{
    _has_hold = false;
}
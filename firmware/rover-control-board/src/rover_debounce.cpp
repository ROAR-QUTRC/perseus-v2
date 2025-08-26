#include "rover_debounce.hpp"

#include <rover_log.hpp>

IoDebouncedButton::IoDebouncedButton(gpio_num_t pin, gpio_pull_mode_t pullMode, bool activeLevel)
    : _activeLevel(activeLevel), _pin(pin)
{
    _lastPressTime = 0;
    ioConfigInput(pin, pullMode);
}

void IoDebouncedButton::handle()
{
    bool pressed       = (gpio_get_level(_pin) == _activeLevel);
    const uint64_t now = coreGetUptime();

    if (pressed)
    {
        if (_lastPressTime == 0)
        {
            // WARN("First press");
            _lastPressTime = now;
        }
        else if ((now - _lastPressTime) < DEBOUNCE_PRESS_TIME)  // capture if not past debounce time
            ;
        else if (!_isPressed)
        {
            if ((now - _lastReleaseTime) < DEBOUNCE_MULTI_PRESS_TIME)
            {
                _repeatPressCount++;
            }
            _isPressed = true;
        }
        else if ((now - _lastPressTime) >= DEBOUNCE_HOLD_TIME)
        {
            if (!_isHeld)
            {
                _hasHold = true;
            }
            _isHeld = true;
        }
    }
    else
    {
        if (_lastPressTime && ((now - _lastPressTime) >= DEBOUNCE_PRESS_TIME))
        {
            _lastPressTime   = 0;
            _lastReleaseTime = now;
            if (!_isHeld && _isPressed)
            {
                _hasPress = true;
            }
            _isPressed = false;
            _isHeld = false;
            clearHasHold();
        }
        else if (_repeatPressCount && (now - _lastReleaseTime) >= DEBOUNCE_MULTI_PRESS_TIME)
        {
            _repeatPressCount = 0;
        }
        _lastPressTime = 0;
    }
}

bool IoDebouncedButton::hasPress()
{
    if (_hasPress)
    {
        clearHasPress();
        return true;
    }
    return false;
}
bool IoDebouncedButton::hasHold()
{
    if (_hasHold)
    {
        clearHasHold();
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
int IoDebouncedButton::getRepeatPressCount()
{
    return _repeatPressCount;
}

void IoDebouncedButton::clearHasPress()
{
    _hasPress = false;
}
void IoDebouncedButton::clearHasHold()
{
    _hasHold = false;
}
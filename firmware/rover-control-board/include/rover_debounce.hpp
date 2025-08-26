#pragma once

#include <driver/gpio.h>
#include <stdint.h>

#include <rover_core.hpp>
#include <rover_io.hpp>

#define DEBOUNCE_PRESS_TIME       5
#define DEBOUNCE_HOLD_TIME        1000UL
#define DEBOUNCE_MULTI_PRESS_TIME 500UL

class IoDebouncedButton
{
public:
    IoDebouncedButton(gpio_num_t pin, gpio_pull_mode_t pullMode = GPIO_PULLDOWN_ONLY, bool activeLevel = false);

    void handle();

    bool hasPress();
    bool hasHold();
    bool isPressed();
    bool isHeld();
    int getRepeatPressCount();
    void clearHasPress();
    void clearHasHold();

private:
    const bool _activeLevel;
    const gpio_num_t _pin;

    int _repeatPressCount     = 0;
    bool _isPressed           = false;
    bool _isHeld              = false;
    bool _hasPress            = false;
    bool _hasHold             = false;
    uint64_t _lastPressTime   = 0;
    uint64_t _lastReleaseTime = 0;
};

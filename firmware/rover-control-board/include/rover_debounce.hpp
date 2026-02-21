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

    bool has_press();
    bool has_hold();
    bool is_pressed();
    bool is_held();
    int get_repeat_press_count();
    void clear_has_press();
    void clear_has_hold();

private:
    const bool _activeLevel;
    const gpio_num_t _pin;

    int _repeat_press_count = 0;
    bool _is_pressed = false;
    bool _is_held = false;
    bool _has_press = false;
    bool _has_hold = false;
    uint64_t _last_press_time = 0;
    uint64_t _last_release_time = 0;
};

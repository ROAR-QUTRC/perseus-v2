#pragma once

#include <cstdint>

#include "current_sensor.hpp"
#include "heater_control.hpp"
#include "servo_driver.hpp"

namespace usb_repl {

/// USB serial REPL for testing without radio
class UsbRepl {
public:
    UsbRepl(servo_driver::ServoDriver& servo, heater_control::HeaterControl& heater,
            current_sensor::CurrentSensor& sensor);

    /// Check for and process a USB serial command. Non-blocking.
    void poll();

private:
    servo_driver::ServoDriver& _servo;
    heater_control::HeaterControl& _heater;
    current_sensor::CurrentSensor& _sensor;

    char _line_buf[128];
    uint8_t _line_pos;

    void process_line(const char* line);
    void cmd_servo(const char* arg);
    void cmd_heater(const char* arg);
    void cmd_stop();
    void cmd_status();
    void cmd_help();
};

}  // namespace usb_repl

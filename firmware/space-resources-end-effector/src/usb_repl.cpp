#include "usb_repl.hpp"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "pico/stdlib.h"

namespace usb_repl
{

    UsbRepl::UsbRepl(servo_driver::ServoDriver& servo,
                     heater_control::HeaterControl& heater,
                     current_sensor::CurrentSensor& sensor)
        : _servo(servo),
          _heater(heater),
          _sensor(sensor),
          _line_pos(0)
    {
        _line_buf[0] = '\0';
    }

    void UsbRepl::poll()
    {
        int ch = getchar_timeout_us(0);
        if (ch == PICO_ERROR_TIMEOUT)
        {
            return;
        }

        if (ch == '\r' || ch == '\n')
        {
            if (_line_pos > 0)
            {
                _line_buf[_line_pos] = '\0';
                printf("\n");
                process_line(_line_buf);
                _line_pos = 0;
                printf("> ");
            }
        }
        else if (ch == '\b' || ch == 127)
        {
            if (_line_pos > 0)
            {
                _line_pos--;
                printf("\b \b");
            }
        }
        else if (_line_pos < sizeof(_line_buf) - 1)
        {
            _line_buf[_line_pos++] = static_cast<char>(ch);
            printf("%c", ch);
        }
    }

    void UsbRepl::process_line(const char* line)
    {
        // Skip leading whitespace
        while (*line && isspace(*line))
        {
            line++;
        }

        if (strncmp(line, "servo ", 6) == 0)
        {
            cmd_servo(line + 6);
        }
        else if (strncmp(line, "heater ", 7) == 0)
        {
            cmd_heater(line + 7);
        }
        else if (strcmp(line, "stop") == 0)
        {
            cmd_stop();
        }
        else if (strcmp(line, "status") == 0)
        {
            cmd_status();
        }
        else if (strcmp(line, "help") == 0)
        {
            cmd_help();
        }
        else
        {
            printf("Unknown command: %s (type 'help' for commands)\n", line);
        }
    }

    void UsbRepl::cmd_servo(const char* arg)
    {
        float speed = strtof(arg, nullptr);
        if (speed < -1.0f)
            speed = -1.0f;
        if (speed > 1.0f)
            speed = 1.0f;
        _servo.set_speed(0, speed);
        printf("Servo speed set to %.3f\n", speed);
    }

    void UsbRepl::cmd_heater(const char* arg)
    {
        int duty = atoi(arg);
        if (duty < 0)
            duty = 0;
        if (duty > 255)
            duty = 255;
        _heater.set_duty(static_cast<uint8_t>(duty));
        printf("Heater duty set to %d\n", duty);
    }

    void UsbRepl::cmd_stop()
    {
        _servo.stop_all();
        _heater.off();
        printf("Emergency stop: servo stopped, heater off\n");
    }

    void UsbRepl::cmd_status()
    {
        float current = _sensor.read_current_amps();
        printf("Current: %.3f A\n", current);
        printf("Heater duty: %u\n", _heater.current_duty());
        printf("Servo driver: %s\n", _servo.is_ok() ? "OK" : "FAULT");
    }

    void UsbRepl::cmd_help()
    {
        printf("Commands:\n");
        printf("  servo <speed>  — Set servo (-1.0 to 1.0)\n");
        printf("  heater <duty>  — Set heater (0-255)\n");
        printf("  stop           — Emergency stop\n");
        printf("  status         — Print telemetry\n");
        printf("  help           — Show this message\n");
    }

}  // namespace usb_repl

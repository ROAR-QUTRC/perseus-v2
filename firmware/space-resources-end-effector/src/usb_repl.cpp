#include "usb_repl.hpp"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "pico/stdlib.h"
#include "protocol.hpp"

namespace usb_repl
{

    UsbRepl::UsbRepl(servo_driver::ServoDriver& servo,
                     heater_control::HeaterControl& heater,
                     current_sensor::CurrentSensor& sensor,
                     const SystemStatus& status,
                     transceiver::Transceiver& radio)
        : _servo(servo),
          _heater(heater),
          _sensor(sensor),
          _status(status),
          _radio(radio),
          _line_pos(0),
          _streaming(false),
          _last_stream_ms(0)
    {
        _line_buf[0] = '\0';
    }

    void UsbRepl::poll()
    {
        // Handle streaming output
        if (_streaming)
        {
            uint32_t now = to_ms_since_boot(get_absolute_time());
            if ((now - _last_stream_ms) >= 200)
            {
                _last_stream_ms = now;
                emit_stream_json();
            }
        }

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
                if (!_streaming)
                {
                    printf("\n");
                }
                process_line(_line_buf);
                _line_pos = 0;
                if (!_streaming)
                {
                    printf("> ");
                }
            }
        }
        else if (ch == '\b' || ch == 127)
        {
            if (_line_pos > 0)
            {
                _line_pos--;
                if (!_streaming)
                {
                    printf("\b \b");
                }
            }
        }
        else if (_line_pos < sizeof(_line_buf) - 1)
        {
            _line_buf[_line_pos++] = static_cast<char>(ch);
            if (!_streaming)
            {
                printf("%c", ch);
            }
        }
    }

    void UsbRepl::process_line(const char* line)
    {
        // Any command stops streaming
        bool was_streaming = _streaming;
        _streaming = false;

        // Skip leading whitespace
        while (*line && isspace(*line))
        {
            line++;
        }

        if (strcmp(line, "stream") == 0)
        {
            cmd_stream();
        }
        else if (strncmp(line, "servo ", 6) == 0)
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
            if (was_streaming)
            {
                printf("Stream stopped.\n");
            }
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
        printf("Current:      %.3f A\n", current);
        printf("Heater duty:  %u\n", _heater.current_duty());
        printf("Servo driver: %s\n", _servo.is_ok() ? "OK" : "FAULT");
        printf("Radio:        %s\n", _status.radio_connected ? "CONNECTED" : "DISCONNECTED");
        if (_status.radio_connected)
        {
            printf("RSSI:         %d dBm\n", _status.rssi_dbm);
        }
        printf("Uptime:       %lu ms\n", (unsigned long)_status.uptime_ms);
        printf("RX commands:  %lu\n", (unsigned long)_status.rx_cmd_count);
        printf("TX telemetry: %lu\n", (unsigned long)_status.tx_telem_count);
        printf("Invalid RX:   %lu\n", (unsigned long)_status.rx_invalid_count);
        if (_status.rx_cmd_count > 0)
        {
            printf("Last command: %s\n", protocol::cmd_type_name(_status.last_cmd_type));
        }
    }

    void UsbRepl::cmd_stream()
    {
        _streaming = true;
        _last_stream_ms = 0;
        printf("Streaming JSON at 5 Hz. Type 'stop' to end.\n");
    }

    void UsbRepl::cmd_help()
    {
        printf("Commands:\n");
        printf("  servo <speed>  — Set servo (-1.0 to 1.0)\n");
        printf("  heater <duty>  — Set heater (0-255)\n");
        printf("  stop           — Emergency stop\n");
        printf("  status         — Print telemetry\n");
        printf("  stream         — Stream JSON status at 5 Hz\n");
        printf("  help           — Show this message\n");
    }

    void UsbRepl::emit_stream_json()
    {
        printf("{\"radio\":%s,\"rssi\":%d,\"ma\":%u,\"servo\":%d,"
               "\"heater\":%u,\"servo_ok\":%s,\"wd\":%s,"
               "\"up\":%lu,\"cmd_age\":%lu,"
               "\"rx\":%lu,\"tx\":%lu,\"bad\":%lu,\"last_cmd\":%u}\n",
               _status.radio_connected ? "true" : "false",
               _status.rssi_dbm,
               _status.current_ma,
               _status.servo_speed,
               _status.heater_duty,
               _status.servo_ok ? "true" : "false",
               _status.watchdog_tripped ? "true" : "false",
               (unsigned long)_status.uptime_ms,
               (unsigned long)_status.last_cmd_age_ms,
               (unsigned long)_status.rx_cmd_count,
               (unsigned long)_status.tx_telem_count,
               (unsigned long)_status.rx_invalid_count,
               _status.last_cmd_type);
    }

}  // namespace usb_repl

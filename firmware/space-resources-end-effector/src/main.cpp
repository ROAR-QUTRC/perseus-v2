#include <cstdio>

#include "current_sensor.hpp"
#include "hardware/i2c.h"
#include "heater_control.hpp"
#include "pico/stdlib.h"
#include "protocol.hpp"
#include "servo_driver.hpp"
#include "system_status.hpp"
#include "transceiver.hpp"
#include "usb_repl.hpp"

// Pin assignments
constexpr uint8_t TRANSCEIVER_SDA = 0;
constexpr uint8_t TRANSCEIVER_SCL = 1;
constexpr uint8_t SERVO_SDA = 2;
constexpr uint8_t SERVO_SCL = 3;
constexpr uint8_t HEATER_GPIO = 15;
constexpr uint8_t CURRENT_ADC_PIN = 26;

// Radio configuration
constexpr uint8_t RADIO_NODE_ID = 2;  // Pico is node 2
constexpr uint8_t RADIO_NETWORK_ID = 100;
constexpr uint8_t RADIO_DEST_NODE = 1;  // Orin is node 1
constexpr uint8_t RADIO_TX_POWER = 13;

// Timing
constexpr uint32_t TELEMETRY_INTERVAL_MS = 200;  // 5 Hz
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 500;
constexpr uint8_t SERVO_CHANNEL = 0;

int main()
{
    stdio_init_all();
    sleep_ms(1000);  // Allow USB enumeration

    printf("\n=== Space Resources End Effector ===\n");
    printf("Initializing...\n");

    // Initialize I2C buses
    // I2C0: transceiver
    // I2C1: PCA9685 servo driver
    transceiver::Transceiver radio(i2c0);
    servo_driver::ServoDriver servo(i2c1);
    current_sensor::CurrentSensor sensor(CURRENT_ADC_PIN);
    heater_control::HeaterControl heater(HEATER_GPIO);

    // Initialize peripherals
    sensor.init();
    heater.init();

    bool radio_ok = radio.init(TRANSCEIVER_SDA, TRANSCEIVER_SCL);
    if (radio_ok)
    {
        radio.configure(RADIO_NODE_ID, RADIO_NETWORK_ID, RADIO_DEST_NODE,
                        RADIO_TX_POWER);
    }
    else
    {
        printf("WARNING: Radio init failed, USB REPL only\n");
    }

    // Initialize servo I2C bus manually (shared with transceiver driver init pattern)
    i2c_init(i2c1, 100000);
    gpio_set_function(SERVO_SDA, GPIO_FUNC_I2C);
    gpio_set_function(SERVO_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(SERVO_SDA);
    gpio_pull_up(SERVO_SCL);
    sleep_ms(10);

    bool servo_ok = servo.init();
    if (!servo_ok)
    {
        printf("WARNING: Servo driver init failed\n");
    }

    // Shared system status for streaming
    SystemStatus sys_status = {};

    // USB REPL for testing
    usb_repl::UsbRepl repl(servo, heater, sensor, sys_status, radio);
    printf("USB REPL ready. Type 'help' for commands.\n> ");

    // State
    uint8_t telemetry_seq = 0;
    int16_t current_servo_speed = 0;
    uint8_t current_heater_duty = 0;
    uint32_t last_command_time = 0;
    uint32_t last_telemetry_time = 0;
    bool watchdog_tripped = false;
    uint32_t rx_cmd_count = 0;
    uint32_t tx_telem_count = 0;
    uint32_t rx_invalid_count = 0;
    uint8_t last_cmd_type = 0;

    // Main loop (cooperative, no RTOS)
    while (true)
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // 1. Check radio for incoming commands
        if (radio.is_connected())
        {
            uint8_t rx_buf[64];
            uint8_t rx_len = radio.receive(rx_buf, sizeof(rx_buf));
            if (rx_len > 0)
            {
                protocol::Command cmd =
                    protocol::decode_command(rx_buf, rx_len);
                if (cmd.valid)
                {
                    last_command_time = now;
                    watchdog_tripped = false;
                    rx_cmd_count++;
                    last_cmd_type = cmd.type;

                    switch (cmd.type)
                    {
                    case protocol::CMD_SET_SERVO:
                        current_servo_speed = cmd.servo_speed;
                        servo.set_speed(SERVO_CHANNEL,
                                        cmd.servo_speed / 1000.0f);
                        break;

                    case protocol::CMD_SET_HEATER:
                        current_heater_duty = cmd.heater_duty;
                        heater.set_duty(cmd.heater_duty);
                        break;

                    case protocol::CMD_SET_ALL:
                        current_servo_speed = cmd.servo_speed;
                        current_heater_duty = cmd.heater_duty;
                        servo.set_speed(SERVO_CHANNEL,
                                        cmd.servo_speed / 1000.0f);
                        heater.set_duty(cmd.heater_duty);
                        break;

                    case protocol::CMD_STOP_ALL:
                        current_servo_speed = 0;
                        current_heater_duty = 0;
                        servo.stop_all();
                        heater.off();
                        break;

                    case protocol::CMD_GET_STATUS:
                    case protocol::CMD_HEARTBEAT:
                        // Just resets watchdog
                        break;

                    default:
                        break;
                    }
                }
                else
                {
                    rx_invalid_count++;
                }
            }
        }

        // 2. Update shared system status
        sys_status.radio_connected = radio.is_connected();
        sys_status.rssi_dbm = radio.is_connected() ? radio.read_rssi() : 0;
        sys_status.current_ma = sensor.read_current_ma();
        sys_status.servo_speed = current_servo_speed;
        sys_status.heater_duty = current_heater_duty;
        sys_status.servo_ok = servo.is_ok();
        sys_status.watchdog_tripped = watchdog_tripped;
        sys_status.uptime_ms = now;
        sys_status.last_cmd_age_ms =
            (last_command_time > 0) ? (now - last_command_time) : 0xFFFFFFFF;
        sys_status.rx_cmd_count = rx_cmd_count;
        sys_status.tx_telem_count = tx_telem_count;
        sys_status.rx_invalid_count = rx_invalid_count;
        sys_status.last_cmd_type = last_cmd_type;

        // 3. Check USB serial for REPL commands
        repl.poll();

        // 4. Send telemetry at 5 Hz
        if (radio.is_connected() && (now - last_telemetry_time) >= TELEMETRY_INTERVAL_MS)
        {
            last_telemetry_time = now;

            uint8_t error_flags = 0;
            if (sys_status.current_ma > 10000)
            {
                error_flags |= protocol::ERR_OVERCURRENT;
            }
            if (watchdog_tripped)
            {
                error_flags |= protocol::ERR_COMM_TIMEOUT;
            }
            if (!servo.is_ok())
            {
                error_flags |= protocol::ERR_SERVO_FAULT;
            }
            if (!radio.is_connected())
            {
                error_flags |= protocol::ERR_TRANSCEIVER_FAULT;
            }

            uint8_t tx_buf[16];
            uint8_t tx_len = protocol::encode_telemetry(
                tx_buf, telemetry_seq++, sys_status.current_ma,
                current_servo_speed, current_heater_duty, error_flags);
            radio.send(tx_buf, tx_len);
            tx_telem_count++;
        }

        // 5. Watchdog: stop all if no command in 500ms
        if (last_command_time > 0 &&
            (now - last_command_time) > WATCHDOG_TIMEOUT_MS && !watchdog_tripped)
        {
            watchdog_tripped = true;
            current_servo_speed = 0;
            current_heater_duty = 0;
            servo.stop_all();
            heater.off();
            printf("WATCHDOG: No command in %ums, stopping all\n",
                   WATCHDOG_TIMEOUT_MS);
        }

        sleep_ms(1);
    }

    return 0;
}

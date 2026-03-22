#include <Arduino.h>
#include <ESP32Servo.h>
#include <FastLED.h>
#include <driver/gpio.h>
#include <math.h>

#include <chrono>
#include <hi_can_twai.hpp>
#include <optional>
#include <string>
#include <vector>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdio.h"

#define DELAY(x) vTaskDelay(x);

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

#define UART_NUM UART_NUM_2

static const int RX_BUF_SIZE = 512;
static const int TX_BUF_SIZE = 512;

using namespace std::chrono_literals;

using namespace hi_can;
using namespace hi_can::addressing;
using namespace hi_can::addressing::post_landing;
using namespace hi_can::addressing::post_landing::arm;
using namespace hi_can::parameters::post_landing::arm::control_board;

std::optional<hi_can::PacketManager> packet_manager;
std::vector<double> current_positions(3, 0);

// map of group to GPIO pin and if its an input (true) or output (false)
Servo servo;
constexpr bool IN = true;  // alias for readability
constexpr bool OUT = false;
std::unordered_map<control_board::pwm_group, std::pair<uint8_t, bool>> pwm_pin_map = {
    {control_board::pwm_group::PWM_1, {6, OUT}},  //  this channel is a special case for the servo
    {control_board::pwm_group::PWM_2, {7, OUT}},
    {control_board::pwm_group::PWM_3, {8, IN}},
    {control_board::pwm_group::PWM_4, {9, IN}}};

const addressing::standard_address_t BASE_ADDRESS{
    SYSTEM_ID,
    arm::SUBSYSTEM_ID,
    arm::control_board::DEVICE_ID};

void register_pwm_device(const control_board::pwm_group& group, bool is_servo = false);
void handle_pwm_data(const Packet& packet, bool is_servo = false);
void register_rsbl_servo(const control_board::rsbl_group& group);
void handle_rsbl_servo_command(const Packet& packet);

void handle_uart(void* args);
void init_uart();
int write_to_motor(uint8_t id, double position, double speed);

#pragma region Main loop

extern "C" void app_main()
{
    // Initialise communications
    init_uart();

    try
    {
        auto& interface = TwaiInterface::get_instance(std::make_pair(GPIO_NUM_2, GPIO_NUM_1), 0,  // tx: 2, rx: 1
                                                      filter_t{
                                                          .address = static_cast<flagged_address_t>(BASE_ADDRESS),
                                                          .mask = DEVICE_MASK,
                                                      });

        packet_manager.emplace(interface);
        printf("CAN interface initialized\n");
    }
    catch (const std::exception& e)
    {
        printf("Error initializing CAN interface: %s\n", e.what());
    }

    // register motors
    register_rsbl_servo(control_board::rsbl_group::SHOULDER_TILT);
    register_rsbl_servo(control_board::rsbl_group::SHOULDER_PAN);
    register_rsbl_servo(control_board::rsbl_group::ELBOW);

    for (const auto& [group, pin_info] : pwm_pin_map)
    {
        if (group == control_board::pwm_group::PWM_1)
        {
            register_pwm_device(group, true);
        }
        else
        {
            register_pwm_device(group);
        }
    }

    printf("Setup complete, entering main loop\n");

    // Start tasks
    xTaskCreate(handle_uart, "UART", 4096, nullptr, configMAX_PRIORITIES - 2, nullptr);

    while (1)
    {
        try
        {
            packet_manager->handle();
        }
        catch (const std::exception& e)
        {
            printf("Error handling CAN packet: %s\n", e.what());
        }

        DELAY(1);
    }
}

#pragma endregion Main loop

#pragma region UART

/*
UART Protocol:
- Start character: <
- End character: >
- Packet label:
    a -> current angles -> <a,tilt_angle,pan_angle,elbow_angle>
    t -> target angles  -> <t,id,target_angle,speed>
- all values are doubles except for id which is a uint8_t
*/

void init_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM, RX_BUF_SIZE * 2, TX_BUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void handle_uart(void* args)
{
    uint8_t* buffer = (uint8_t*)malloc(RX_BUF_SIZE);
    while (true)
    {
        int len = uart_read_bytes(UART_NUM, buffer, RX_BUF_SIZE, 0);

        std::string s = "";

        if (len > 0)
        {
            for (int i = 0; i < len; i++)
            {
                uint8_t byte = buffer[i];
                if (byte == '<')
                {  // Start byte so reset the string
                    s = "";
                }
                else if (byte == '>')
                {
                    // parse angle message
                    if (!s.empty() && s[0] == 'a')
                    {
                        int tilt_end = s.find(',', 2);
                        int pan_end = s.find(',', tilt_end + 1);
                        if (tilt_end == std::string::npos || pan_end == std::string::npos)
                        {
                            printf("Invalid message format: %s\n", s.c_str());
                        }
                        else
                        {
                            double tilt_angle = std::stod(s.substr(2, tilt_end - 2));
                            double pan_angle = std::stod(s.substr(tilt_end + 1, pan_end - tilt_end - 1));
                            double elbow_angle = std::stod(s.substr(pan_end + 1));
                            current_positions[0] = tilt_angle;
                            current_positions[1] = pan_angle;
                            current_positions[2] = elbow_angle;
                        }
                    }
                }
                else
                {
                    s += (char)byte;
                }
            }
        }
        DELAY(1);
    }
    free(buffer);
}

int write_to_motor(uint8_t id, double position, double speed)
{
    std::string data = "<t," + std::to_string(id) + "," + std::to_string(position) + "," + std::to_string(speed) + ">";
    // printf("<t,%s,%s,%s>\n", std::to_string(id).c_str(), std::to_string(position).c_str(), std::to_string(speed).c_str());
    return uart_write_bytes(UART_NUM, data.c_str(), strlen(data.c_str()));
}

#pragma endregion UART

#pragma region RSBL Servo

void handle_rsbl_servo_command(const Packet& packet)
{
    using namespace control_board;

    try
    {
        // get target device group from address
        uint8_t group_id = static_cast<standard_address_t>(
                               packet.get_address().address)
                               .group;
        // Get the command type from the parameter ID
        control_board::rsbl_parameters command_type = static_cast<control_board::rsbl_parameters>(
            static_cast<standard_address_t>(
                packet.get_address().address)
                .parameter);

        // action command
        switch (command_type)
        {
        case rsbl_parameters::SET_POS_EX:
        {
            const position_control_t position_control_cmd = position_control_t{packet.get_data()};
            if (group_id > 3)
            {
                printf("Received RSBL command for invalid group %d\n", group_id);
                return;
            }
            write_to_motor(group_id, position_control_cmd.position, position_control_cmd.acceleration);
            // printf("Target position: %d, speed: %d, acceleration: %d\n",
            //               position_control_cmd.position,
            //               position_control_cmd.duration_ms,
            //               position_control_cmd.acceleration);
            break;
        }
        case rsbl_parameters::SET_POSITION_SINGLE:
        {
            const int16_t position_cmd = position_t{packet.get_data()}.value;
            printf("SET_POSITION_SINGLE command not implemented. Target position: %d\n", position_cmd);
            break;
        }
        case rsbl_parameters::SET_SPEED:
        {
            const uint16_t speed_cmd = speed_t{packet.get_data()}.value;
            printf("SET_SPEED command not implemented. Target speed: %d\n", speed_cmd);
            break;
        }
        case rsbl_parameters::SET_TORQUE_ENABLE:
        {
            const bool torque_enable = torque_enable_t{packet.get_data()}.value;
            printf("SET_TORQUE_ENABLE command not implemented. Target: %d\n", torque_enable);
            break;
        }
        case rsbl_parameters::STATUS_1:
        case rsbl_parameters::STATUS_2:
            // Not a command we handle
            break;
        default:
            printf("Unknown RSBL command type: %d\n", static_cast<uint8_t>(command_type));
            break;
        }
    }
    catch (const std::exception& e)
    {
        printf("Failed to parse RSBL servo packet: %s\n", e.what());
    }
}

void register_rsbl_servo(const control_board::rsbl_group& group)
{
    standard_address_t address{
        BASE_ADDRESS,
        static_cast<uint8_t>(group), 0};  // Parameter ignored by mask

    packet_manager->set_callback(
        filter_t{.address = static_cast<flagged_address_t>(address),
                 // Mask out parameter ID to catch all RSBL servo commands
                 .mask = 0xFFFFFF00},
        {.data_callback = handle_rsbl_servo_command});

    const uint8_t ID = static_cast<uint8_t>(group);

    address.parameter = static_cast<uint8_t>(control_board::rsbl_parameters::STATUS_1);
    packet_manager->set_transmission_config(static_cast<flagged_address_t>(address),
                                            {.generator = [=]()
                                             {
                                                 // static int posCount = 0;
                                                 // posCount++;
                                                 // printf("Generating status packet with position %d\n", posCount);
                                                 status_1_t status{};  // TODO: get actual position
                                                 // status.position = static_cast<int16_t>(posCount);
                                                 status.position = static_cast<int16_t>(current_positions[ID - 1]);
                                                 //  status.speed = static_cast<int16_t>(servo.ReadSpeed(ID));
                                                 //  status.load = static_cast<int16_t>(servo.ReadLoad(ID));
                                                 //  status.position = static_cast<int16_t>(123);
                                                 status.speed = static_cast<int16_t>(0);
                                                 status.load = static_cast<int16_t>(0);
                                                 return status.serialize_data();
                                             },
                                             .interval = 100ms});

    address.parameter = static_cast<uint8_t>(control_board::rsbl_parameters::STATUS_2);
    packet_manager->set_transmission_config(static_cast<flagged_address_t>(address),
                                            {.generator = [=]()
                                             {
                                                 status_2_t status{};  // TODO: get actual data
                                                 status.voltage = static_cast<uint16_t>(123);
                                                 status.temperature = static_cast<int8_t>(345);
                                                 status.current = static_cast<uint16_t>(456);
                                                 status.moving = static_cast<uint8_t>(567);
                                                 //  status.voltage = static_cast<uint16_t>(servo.ReadVoltage(ID));
                                                 //  status.temperature = static_cast<int8_t>(servo.ReadTemper(ID));
                                                 //  status.current = static_cast<uint16_t>(servo.ReadCurrent(ID));
                                                 //  status.moving = static_cast<uint8_t>(servo.ReadMove(ID));
                                                 return status.serialize_data();
                                             },
                                             .interval = 1000ms});
}

#pragma endregion RSBL Servo

#pragma region PWM Device

void handle_pwm_data(const Packet& packet, bool is_servo)
{
    try
    {
        // get target device group from address
        control_board::pwm_group group_id = static_cast<control_board::pwm_group>(
            static_cast<standard_address_t>(
                packet.get_address().address)
                .group);
        uint16_t pwm_value = pwm_t{packet.get_data()}.value;

        // set pwm value
        if (!pwm_pin_map[group_id].second)
        {
            // output
            if (is_servo)
            {
                // map 16-bit value to servo angle (0-180)
                double angle = (pwm_value / 65535.0) * 180.0;
                servo.write(angle);
            }
            else
            {
                uint16_t duty = static_cast<uint16_t>((pwm_value / 65535.0) * 255);  // map 16-bit value to 8-bit duty cycle
                analogWrite(pwm_pin_map[group_id].first, duty);
            }
        }
    }
    catch (const std::exception& e)
    {
        printf("Failed to parse PWM packet: %s\n", e.what());
    }
}

void register_pwm_device(const control_board::pwm_group& group, bool is_servo)
{
    standard_address_t address{
        BASE_ADDRESS,
        static_cast<uint8_t>(group),
        static_cast<uint8_t>(control_board::pwm_parameters::SET_PWM)};

    if (!pwm_pin_map[group].second)
    {
        packet_manager->set_callback(
            filter_t{static_cast<flagged_address_t>(address)},
            {.data_callback = [=](const Packet& p)
             { handle_pwm_data(p, is_servo); }});

        if (is_servo)
        {
            // attach servo to pin
            servo.attach(pwm_pin_map[group].first);
            servo.write(0);  // start at 0 degrees
        }
        else
        {
            pinMode(pwm_pin_map[group].first, OUTPUT);
        }
    }
    else
    {
        pinMode(pwm_pin_map[group].first, INPUT);

        address.parameter = static_cast<uint8_t>(control_board::pwm_parameters::GET_ANALOG);

        packet_manager->set_transmission_config(static_cast<flagged_address_t>(address),
                                                {.generator = [=]()
                                                 {
                                                     pwm_t pwm{};
                                                     int reading = analogRead(pwm_pin_map[group].first);
                                                     pwm.value = static_cast<uint16_t>((reading / 255.0) * 65535);  // map 8-bit reading to 16-bit value
                                                     return pwm.serialize_data();
                                                 },
                                                 .interval = 100ms});
    }
}

#pragma endregion PWM Device
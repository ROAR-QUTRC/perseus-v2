#include <Arduino.h>
#include <math.h>

#include <chrono>
#include <optional>
#include <vector>
#include <chrono>
#include <hi_can_twai.hpp>

#include "RSB.h"

using namespace std::chrono_literals;

using namespace hi_can;
using namespace hi_can::addressing;
using namespace hi_can::addressing::post_landing;
using namespace hi_can::addressing::post_landing::arm;
using namespace hi_can::parameters::post_landing::arm::control_board;

#define MAX_ENCODER_COUNT 4095

SMS_STS servo;

const int TILT = static_cast<int>(control_board::group::SHOULDER_TILT);
const int PAN = static_cast<int>(control_board::group::SHOULDER_PAN);
const int ELBOW = static_cast<int>(control_board::group::ELBOW);

std::vector<int> past_positions(3, 0);
std::vector<double> target_positions = {25.0 * 0.0, 25.0 * 0.0, 25 * 0.0};
std::optional<hi_can::PacketManager> packet_manager;

const addressing::standard_address_t BASE_ADDRESS{
    SYSTEM_ID,
    arm::SUBSYSTEM_ID,
    arm::control_board::DEVICE_ID};

void register_pwm_device(const control_board::group& group);
void handle_pwm_data(const Packet& packet);
void register_rsbl_servo(const control_board::group& group);
void handle_rsbl_servo_command(const Packet& packet);

void setup()
{
    Serial.begin(115200);
    Serial.println("RSBL85 CAN Bridge - Initializing");

    servo.begin(1000000, 18, 19, -1);
    delay(1000);

    Serial.println("Motor 1 online: " + String(TILT == servo.Ping(TILT) ? "true" : "false"));
    Serial.println("Motor 2 online: " + String(PAN == servo.Ping(PAN) ? "true" : "false"));
    Serial.println("Motor 3 online: " + String(ELBOW == servo.Ping(ELBOW) ? "true" : "false"));

    past_positions[TILT] = servo.ReadPos(TILT);
    past_positions[PAN] = servo.ReadPos(PAN);
    past_positions[ELBOW] = servo.ReadPos(ELBOW);
    
    auto& interface = TwaiInterface::get_instance(std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
                                                  filter_t{
                                                      .address = static_cast<flagged_address_t>(BASE_ADDRESS),
                                                      .mask = DEVICE_MASK,
                                                  });

    packet_manager.emplace(interface);
    Serial.println("CAN interface initialized");

    register_pwm_device(control_board::group::PWM_1);
    register_pwm_device(control_board::group::PWM_2);
    register_rsbl_servo(control_board::group::SHOULDER_TILT);
    register_rsbl_servo(control_board::group::SHOULDER_PAN);
    register_rsbl_servo(control_board::group::ELBOW);
}

std::vector<double> positions(3, 0);
std::vector<int16_t> full_rev_count(3, 0);
void write_angle(int id, double angle)
{
    target_positions[id] = angle;
}

const int ids[] = {TILT, PAN, ELBOW};

void loop()
{
    try
    {
        packet_manager->handle();
    }
    catch (const std::exception& e)
    {
        Serial.println("Error handling CAN packet: " + String(e.what()));
    }

    #pragma region Handle motor control

    for (unsigned i = 0; i < 3; i++)
    {
        const int id = ids[i];
        const int read_pos = servo.ReadPos(id);
        const int diff = read_pos - past_positions[id];

        if (abs(diff) > 2000)
        {  // If the jump is greater than half a revolution then the encoder has wrapped around
            bool clockwise = diff < 0;
            full_rev_count[id] += clockwise ? 1 : -1;
        }

        past_positions[id] = read_pos;

        const bool negative = full_rev_count[id] < 0;
        const int raw_angle = MAX_ENCODER_COUNT * ((negative ? 1 : 0) + full_rev_count[id]) + (negative ? (MAX_ENCODER_COUNT - read_pos) * -1 : read_pos);
        positions[id] = ((double)raw_angle / (double)MAX_ENCODER_COUNT);
        positions[id] *= 360.0;  // degrees
        // positions[id] *= PI * 2.0; // radians

        constexpr double MAX_SPEED = 30000.0;
        const double error = target_positions[id] - positions[id];

        if (abs(error) <= 1)
        {
            servo.WriteSpe(id, 0, 0);
        }
        else if (abs(error) < 360)
        {  // Reduce speed when 1 revolution from target
            servo.WriteSpe(id, error / 360.0 * MAX_SPEED, 0);
        }
        else 
        {
            servo.WriteSpe(id, error > 0 ? MAX_SPEED : -MAX_SPEED, 0);
        }
    }

    #pragma endregion Handle motor control
}

#pragma region RSBL Servo

void handle_rsbl_servo_command(const Packet& packet)
{
    using namespace control_board;

    try
    {
        // get target device group from address
        uint8_t group_id = static_cast<standard_address_t>(
                packet.get_address().address).group;
        // Get the command type from the parameter ID
        control_board::rsbl_parameters command_type = static_cast<control_board::rsbl_parameters>(
            static_cast<standard_address_t>(
                packet.get_address().address)
                .parameter);

        Serial.printf("Target device group: %d\n", group_id);

        // action command
        switch (command_type)
        {
        case rsbl_parameters::SET_POS_EX:
        {
            const position_control_t position_control_cmd = position_control_t{packet.get_data()};
            if (group_id > 2) 
            {
                Serial.printf("Received RSBL command for invalid group %d\n", group_id);
                return;
            }
            target_positions[group_id] = position_control_cmd.position;
            Serial.printf("Target position: %d, speed: %d, acceleration: %d\n",
                          position_control_cmd.position,
                          position_control_cmd.duration_ms,
                          position_control_cmd.acceleration);
            break;
        }
        case rsbl_parameters::SET_POSITION_SINGLE:
        {
            const int16_t position_cmd = position_t{packet.get_data()}.value;
            Serial.printf("SET_POSITION_SINGLE command not implemented. Target position: %d\n", position_cmd);
            break;
        }
        case rsbl_parameters::SET_SPEED:
        {
            const uint16_t speed_cmd = speed_t{packet.get_data()}.value;
            Serial.printf("SET_SPEED command not implemented. Target speed: %d\n", speed_cmd);
            break;
        }
        case rsbl_parameters::SET_TORQUE_ENABLE:
        {
            const bool torque_enable = torque_enable_t{packet.get_data()}.value;
            Serial.printf("SET_TORQUE_ENABLE command not implemented. Target: %d\n", torque_enable);
            break;
        }
        case rsbl_parameters::STATUS_1:
        case rsbl_parameters::STATUS_2:
            // Not a command we handle
            break;
        default:
            Serial.printf("Unknown RSBL command type: %d\n", static_cast<uint8_t>(command_type));
            break;
        }
    }
    catch (const std::exception& e)
    {
        Serial.printf("Failed to parse RSBL servo packet: %s\n", e.what());
    }
}

void register_rsbl_servo(const control_board::group& group)
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
                                                 status_1_t status{};  // TODO: get actual position
                                                 status.position = static_cast<int16_t>(positions[ID]);
                                                 status.speed = static_cast<int16_t>(servo.ReadSpeed(ID));
                                                 status.load = static_cast<int16_t>(servo.ReadLoad(ID));
                                                 return status.serialize_data();
                                             },
                                             .interval = 100ms});

    address.parameter = static_cast<uint8_t>(control_board::rsbl_parameters::STATUS_2);
    packet_manager->set_transmission_config(static_cast<flagged_address_t>(address),
                                            {.generator = [=]()
                                             {
                                                 status_2_t status{};  // TODO: get actual data
                                                 status.voltage = static_cast<uint16_t>(servo.ReadVoltage(ID));
                                                 status.temperature = static_cast<int8_t>(servo.ReadTemper(ID));
                                                 status.current = static_cast<uint16_t>(servo.ReadCurrent(ID));
                                                 status.moving = static_cast<uint8_t>(servo.ReadMove(ID));
                                                 return status.serialize_data();
                                             },
                                             .interval = 1000ms});
}

#pragma endregion RSBL Servo

#pragma region PWM Device

void handle_pwm_data(const Packet& packet)
{
    try
    {
        // get target device group from address
        control_board::group group_id = static_cast<control_board::group>(
            static_cast<standard_address_t>(
                packet.get_address().address)
                .group);
        uint16_t pwm_value = pwm_t{packet.get_data()}.value;
        // set pwm value
        if (group_id == control_board::group::PWM_1)
        {
            Serial.printf("Received PWM_1 command: %d\n", pwm_value);
        }
        else if (group_id == control_board::group::PWM_2)
        {
            Serial.printf("Received PWM_2 command: %d\n", pwm_value);
        }
    }
    catch (const std::exception& e)
    {
        Serial.printf("Failed to parse PWM packet: %s\n", e.what());
    }
}

void register_pwm_device(const control_board::group& group)
{
    const standard_address_t address{
        BASE_ADDRESS,
        static_cast<uint8_t>(group),
        static_cast<uint8_t>(control_board::pwm_parameters::SET_PWM)};

    packet_manager->set_callback(
        filter_t{static_cast<flagged_address_t>(address)},
        {.data_callback = handle_pwm_data});
}

#pragma endregion PWM Device
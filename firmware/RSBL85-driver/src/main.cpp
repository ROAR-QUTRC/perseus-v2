#include <Arduino.h>

#include <chrono>
#include <hi_can_parameter.hpp>
#include <hi_can_twai.hpp>
#include <optional>

#include "RSB.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace hi_can;
using namespace hi_can::addressing;
using namespace hi_can::addressing::post_landing::arm::control_board;
using namespace hi_can::parameters::post_landing::arm::control_board;

SMS_STS servo;
std::optional<PacketManager> packet_manager;

// Base address for the control board
const standard_address_t BASE_ADDRESS{
    post_landing::SYSTEM_ID,
    post_landing::arm::SUBSYSTEM_ID,
    DEVICE_ID};

void setup()
{
    Serial.begin(115200);
    Serial.println("RSBL85 CAN Bridge - Initializing");

    servo.begin(1000000, 18, 19, -1);
    delay(1000);

    for (uint8_t id = 0; id <= 1; id++)  // Servo Setup - Configure both servos (IDs 0 and 1 for SHOULDER)
    {
        servo.unLockEprom(id);
        servo.LockEprom(id);
        servo.writeByte(id, SMS_STS_MODE, 3);  // Step mode
        servo.EnableTorque(id, 1);
        delay(100);
    }

    auto& can_interface = TwaiInterface::get_instance(std::make_pair(GPIO_NUM_21, GPIO_NUM_42));
    packet_manager.emplace(can_interface);
    Serial.println("CAN interface initialized");

    // Map servo index to group ID
    group servo_groups[] = {group::SHOULDER_TILT, group::SHOULDER_PAN};

    for (uint8_t servo_idx = 0; servo_idx <= 1; servo_idx++)
    {
        group servo_group = servo_groups[servo_idx];

        // Register SET_POS_EX callback (position control command)
        packet_manager->set_callback(
            filter_t{
                .address = static_cast<flagged_address_t>(standard_address_t(
                    BASE_ADDRESS,
                    static_cast<uint8_t>(servo_group),
                    static_cast<uint8_t>(rsbl_parameters::SET_POS_EX)))},
            PacketManager::callback_config_t{
                .data_callback = [servo_idx](const Packet& packet)
                {
                    try
                    {
                        auto data = packet.get_data<_position_control_t>();
                        if (!data.has_value())
                        {
                            Serial.println("Error: Invalid SET_POS_EX data size");
                            return;
                        }

                        int16_t position = data->position;
                        uint16_t speed = data->duration_ms;  // Note: driver sends speed in this field
                        uint8_t acceleration = data->acceleration;

                        // servo.MovetoAngle(servo_idx, position, speed, acceleration);
                        servo.WritePosEx(servo_idx, position, speed, acceleration);  // could swap out to MovetoAngle
                        Serial.printf("SET_POS_EX: servo=%d, pos=%d, spd=%d, acc=%d\n",
                                      servo_idx, position, speed, acceleration);
                    }
                    catch (...)
                    {
                        Serial.println("Error parsing SET_POS_EX");
                    }
                }});

        // Register STATUS_1 callback (responds with position, speed, load)
        packet_manager->set_callback(
            filter_t{
                .address = static_cast<flagged_address_t>(standard_address_t(
                    BASE_ADDRESS,
                    static_cast<uint8_t>(servo_group),
                    static_cast<uint8_t>(rsbl_parameters::STATUS_1)))},
            PacketManager::callback_config_t{
                .data_callback = [servo_idx, servo_group](const Packet& packet)
                {
                    int16_t position = servo.ReadPos(servo_idx);
                    int16_t speed = servo.ReadSpeed(servo_idx);
                    int16_t load = servo.ReadLoad(servo_idx);

                    _status_1_t status;
                    status.position = position;
                    status.speed = speed;
                    status.load = load;

                    auto& can_interface = TwaiInterface::get_instance();
                    can_interface.transmit(Packet(
                        static_cast<flagged_address_t>(standard_address_t(
                            BASE_ADDRESS,
                            static_cast<uint8_t>(servo_group),
                            static_cast<uint8_t>(rsbl_parameters::STATUS_1))),
                        status));

                    Serial.printf("STATUS_1 sent: servo=%d, pos=%d, speed=%d, load=%d\n",
                                  servo_idx, position, speed, load);
                }});

        // Register STATUS_2 callback (responds with voltage, temp, current, moving)
        packet_manager->set_callback(
            filter_t{
                .address = static_cast<flagged_address_t>(standard_address_t(
                    BASE_ADDRESS,
                    static_cast<uint8_t>(servo_group),
                    static_cast<uint8_t>(rsbl_parameters::STATUS_2)))},
            PacketManager::callback_config_t{
                .data_callback = [servo_idx, servo_group](const Packet& packet)
                {
                    uint16_t voltage = servo.ReadVoltage(servo_idx);
                    int8_t temperature = servo.ReadTemperature(servo_idx);
                    uint16_t current = servo.ReadCurrent(servo_idx);
                    bool moving = servo.IsMoving(servo_idx);

                    _status_2_t status;
                    status.voltage = voltage;
                    status.temperature = temperature;
                    status.current = current;
                    status.moving = moving ? 1 : 0;

                    auto& can_interface = TwaiInterface::get_instance();
                    can_interface.transmit(Packet(
                        static_cast<flagged_address_t>(standard_address_t(
                            BASE_ADDRESS,
                            static_cast<uint8_t>(servo_group),
                            static_cast<uint8_t>(rsbl_parameters::STATUS_2))),
                        status));

                    Serial.printf("STATUS_2 sent: servo=%d, volt=%d, temp=%d, curr=%d, moving=%d\n",
                                  servo_idx, voltage, temperature, current, moving);
                }});
    }

    // Register PWM callbacks
    packet_manager->set_callback(
        filter_t{
            .address = static_cast<flagged_address_t>(standard_address_t(
                BASE_ADDRESS,
                static_cast<uint8_t>(group::PWM_1),
                static_cast<uint8_t>(pwm_parameters::SET_PWM)))},
        PacketManager::callback_config_t{
            .data_callback = [](const Packet& packet)
            {
                auto data = packet.get_data<uint16_t>();
                if (data.has_value())
                {
                    Serial.printf("PWM1: value=%d\n", *data);
                    // TODO: Set PWM output
                }
            }});

    packet_manager->set_callback(
        filter_t{
            .address = static_cast<flagged_address_t>(standard_address_t(
                BASE_ADDRESS,
                static_cast<uint8_t>(group::PWM_2),
                static_cast<uint8_t>(pwm_parameters::SET_PWM)))},
        PacketManager::callback_config_t{
            .data_callback = [](const Packet& packet)
            {
                auto data = packet.get_data<uint16_t>();
                if (data.has_value())
                {
                    Serial.printf("PWM2: value=%d\n", *data);
                    // TODO: Set PWM output
                }
            }});

    Serial.println("Setup complete - ready to receive CAN commands");
}

void loop()
{
    static unsigned long last_debug = 0;

    try
    {
        packet_manager->handle();
    }
    catch (const std::exception& e)
    {
        Serial.printf("CAN error: %s\n", e.what());
    }

    // Debug: print status every 2 seconds
    if (millis() - last_debug > 2000)
    {
        last_debug = millis();
        Serial.println("Waiting for CAN messages...");
    }

    // TODO: Control PWM output pin for PWM_1
}

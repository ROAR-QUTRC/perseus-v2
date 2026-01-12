#include <Arduino.h>

#include <chrono>
#include <hi_can_parameter.hpp>
#include <hi_can_twai.hpp>
#include <optional>

#include "RSB.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace hi_can;
using namespace hi_can::addressing::post_landing::servo::servo_board;
using namespace hi_can::parameters::post_landing::servo::servo_board;
using namespace hi_can::parameters::post_landing::servo::servo_board::send_message;
using namespace hi_can::parameters::post_landing::servo::servo_board::receive_message;
using hi_can::parameters::SimpleSerializable;

SMS_STS servo;
std::optional<PacketManager> packet_manager;

void setup()
{
    Serial.begin(115200);
    Serial.println("RSBL85 CAN Bridge - Initializing");

    servo.begin(1000000, 18, 19, -1);
    delay(1000);

    for (uint8_t id = 0; id <= 1; id++)  // Servo Setup - Configure both servos (IDs 0 and 1 for SHOULDER)
    {
        servo.unLockEprom(id);
        servo.writeByte(id, SMS_STS_MODE, 3);  // Step mode
        servo.LockEprom(id);
        servo.EnableTorque(id, 1);
        delay(100);
    }

    auto& can_interface = TwaiInterface::get_instance(std::make_pair(GPIO_NUM_21, GPIO_NUM_22));  // initialize TWAI pins
    packet_manager.emplace(can_interface);
    Serial.println("CAN interface initialized");

    using namespace addressing;

    servo_id_t move_servo_ids[] = {servo_id_t::SHOULDER_TILT, servo_id_t::SHOULDER_PAN};  // Register MOVE_TO_ANGLE callbacks for both servos
    for (uint8_t servo_idx = 0; servo_idx <= 1; servo_idx++)
    {
        servo_id_t servo_id = move_servo_ids[servo_idx];

        packet_manager->set_callback(
            filter_t{
                .address = static_cast<flagged_address_t>(servo_address_t(static_cast<uint8_t>(command_t::MOVE_TO_ANGLE), servo_id))},
            PacketManager::callback_config_t{
                .data_callback = [servo_idx](const Packet& packet)
                {
                    try
                    {
                        SimpleSerializable<move_to_angle_raw_t> cmd(packet.get_data());

                        int16_t angle = cmd.angle;
                        uint16_t time_ms = cmd.time_ms;
                        uint8_t acceleration = cmd.acceleration;

                        // TESTING uncomment HERE
                        // servo.moveToAngle(servo_idx, angle, time_ms, acceleration);

                        Serial.printf("MOVE_TO_ANGLE: servo=%d, angle=%d, time=%d, acc=%d\n",
                                      servo_idx, angle, time_ms, acceleration);
                    }
                    catch (...)
                    {
                        Serial.println("Error parsing MOVE_TO_ANGLE");
                    }
                }});

        packet_manager->set_callback(
            filter_t{
                .address = static_cast<flagged_address_t>(servo_address_t(static_cast<uint8_t>(command_t::WRITE_POS_EX), servo_id))},
            PacketManager::callback_config_t{
                .data_callback = [servo_idx](const Packet& packet)
                {
                    try
                    {
                        SimpleSerializable<write_pos_ex_t> cmd(packet.get_data());

                        int16_t position = cmd.position;
                        uint16_t speed = cmd.speed;
                        uint8_t acceleration = cmd.acceleration;

                        // servo.WritePosEx(servo_idx, position, speed, acceleration);
                        Serial.printf("WRITE_POS_EX: servo=%d, pos=%d, spd=%d, acc=%d\n",
                                      servo_idx, position, speed, acceleration);
                    }
                    catch (...)
                    {
                        Serial.println("Error parsing WRITE_POS_EX");
                    }
                }});
    }

    packet_manager->set_callback(  // Register callback for PWM_1
        filter_t{
            .address = static_cast<flagged_address_t>(servo_address_t(static_cast<uint8_t>(command_t::SET_PWM_VALUES), servo_id_t::PWM_1))},
        PacketManager::callback_config_t{
            .data_callback = [](const Packet& packet)
            {
                try
                {
                    SimpleSerializable<set_pwm_raw_t> cmd(packet.get_data());
                    uint16_t pwm = cmd.pwm;
                    Serial.printf("PWM1: value=%d\n", pwm);
                }
                catch (...)
                {
                }
            }});

    packet_manager->set_callback(  // Register callback for PWM_2
        filter_t{
            .address = static_cast<flagged_address_t>(servo_address_t(static_cast<uint8_t>(command_t::SET_PWM_VALUES), servo_id_t::PWM_2))},
        PacketManager::callback_config_t{
            .data_callback = [](const Packet& packet)
            {
                try
                {
                    SimpleSerializable<set_pwm_raw_t> cmd(packet.get_data());
                    uint16_t pwm = cmd.pwm;
                    Serial.printf("PWM2: value=%d\n", pwm);
                }
                catch (...)
                {
                }
            }});

    servo_id_t servo_ids[] = {servo_id_t::SHOULDER_TILT, servo_id_t::SHOULDER_PAN};  // Register READ_STATUS_1 callback for both servos
    for (uint8_t servo_idx = 0; servo_idx <= 1; servo_idx++)
    {
        servo_id_t servo_id = servo_ids[servo_idx];

        packet_manager->set_callback(
            filter_t{
                .address = static_cast<flagged_address_t>(servo_address_t(static_cast<uint8_t>(command_t::READ_STATUS_1), servo_id))},
            PacketManager::callback_config_t{
                .data_callback = [servo_idx, servo_id](const Packet& packet) {  // Read servo status
                    // TEST - mock values
                    int16_t position = 450 + (servo_idx * 100);  // servo 0: 450, servo 1: 550
                    int16_t speed = 50;
                    int16_t load = 200;

                    // int16_t position = servo.ReadPos(servo_idx);
                    // int16_t speed = servo.ReadSpeed(servo_idx);
                    // int16_t load = servo.ReadLoad(servo_idx);

                    status_1_t status;
                    status.position = position;
                    status.speed = speed;
                    status.load = load;

                    SimpleSerializable<status_1_t> raw_status(status);

                    auto& can_interface = TwaiInterface::get_instance();
                    can_interface.transmit(Packet(
                        static_cast<flagged_address_t>(servo_address_t(static_cast<uint8_t>(command_t::READ_STATUS_1), servo_id)),
                        raw_status.serialize_data()));

                    Serial.printf("STATUS_1 sent: servo=%d, pos=%d, speed=%d, load=%d\n",
                                  servo_idx, position, speed, load);
                }});

        packet_manager->set_callback(  // Register READ_STATUS_2 callback
            filter_t{
                .address = static_cast<flagged_address_t>(servo_address_t(static_cast<uint8_t>(command_t::READ_STATUS_2), servo_id))},
            PacketManager::callback_config_t{
                .data_callback = [servo_idx, servo_id](const Packet& packet)
                {
                    // TEST - mock values
                    uint16_t voltage = 12000;  // 12.0V in millivolts
                    int8_t temperature = 35;   // 35°C
                    int16_t current = 100;     // 100mA
                    bool moving = false;

                    status_2_t status;
                    status.voltage = voltage;
                    status.temperature = temperature;
                    status.current = current;
                    status.moving = moving ? 1 : 0;

                    SimpleSerializable<status_2_t> raw_status(status);

                    auto& can_interface = TwaiInterface::get_instance();
                    can_interface.transmit(Packet(
                        static_cast<flagged_address_t>(servo_address_t(static_cast<uint8_t>(command_t::READ_STATUS_2), servo_id)),
                        raw_status.serialize_data()));

                    Serial.printf("STATUS_2 sent: servo=%d, volt=%d, temp=%d, curr=%d, moving=%d\n",
                                  servo_idx, voltage, temperature, current, moving);
                }});
    }

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

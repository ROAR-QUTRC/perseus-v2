#include <chrono>
#include <hi_can_parameter.hpp>
#include <hi_can_raw.hpp>
#include <iostream>
#include <optional>
#include <thread>

using namespace hi_can;
using namespace hi_can::addressing;
using namespace hi_can::addressing::post_landing;
using namespace hi_can::addressing::post_landing::arm;
using namespace hi_can::parameters::post_landing::arm::control_board;
// using namespace hi_can::addressing::post_landing::servo::servo_board;
// using namespace hi_can::parameters::post_landing::servo::servo_board;
// using namespace hi_can::parameters::post_landing::servo::servo_board::send_message;
// using namespace hi_can::parameters::post_landing::servo::servo_board::receive_message;
// using hi_can::parameters::SimpleSerializable;

std::optional<hi_can::RawCanInterface> can_interface;
std::optional<hi_can::PacketManager> packet_manager;

void register_pwm_device(const control_board::group& group);
void handle_pwm_data(const Packet& packet);

const addressing::standard_address_t base_address{
    SYSTEM_ID,
    arm::SUBSYSTEM_ID,
    arm::control_board::DEVICE_ID };

void setup() {
    can_interface.emplace(RawCanInterface("tyler"));
    packet_manager.emplace(can_interface.value());
    std::cout << "CAN interface initialized on vcan0" << std::endl;

    // using namespace addressing;

    register_pwm_device(control_board::group::PWM_1);
    register_pwm_device(control_board::group::PWM_2);

    // motor_id_t move_servo_ids[] = {motor_id_t::SHOULDER_TILT, motor_id_t::SHOULDER_PAN};  // Register MOVE_TO_ANGLE callbacks for both servos
    // for (uint8_t servo_idx = 0; servo_idx <= 1; servo_idx++)
    // {
    //     motor_id_t servo_id = move_servo_ids[servo_idx];

    //     packet_manager->set_callback(
    //         filter_t{
    //             .address = static_cast<flagged_address_t>(servo_address_t(static_cast<uint8_t>(command_t::MOVE_TO_ANGLE), servo_id))},
    //         PacketManager::callback_config_t{
    //             .data_callback = [servo_idx](const Packet& packet)
    //             {
    //                 try
    //                 {
    //                     SimpleSerializable<move_to_angle_raw_t> cmd(packet.get_data());

    //                     int16_t angle = cmd.angle;
    //                     uint16_t time_ms = cmd.time_ms;
    //                     uint8_t acceleration = cmd.acceleration;

    //                     // TESTING uncomment HERE
    //                     // servo.moveToAngle(servo_idx, angle, time_ms, acceleration);

    //                     std::cout << "MOVE_TO_ANGLE: servo=" << static_cast<int>(servo_idx)
    //                               << ", angle=" << angle
    //                               << ", time=" << time_ms
    //                               << ", acc=" << static_cast<int>(acceleration) << std::endl;
    //                 }
    //                 catch (const std::exception& e)
    //                 {
    //                     std::cout << "Exception: " << e.what() << std::endl;
    //                 }
    //             }});

    //     packet_manager->set_callback(
    //         filter_t{
    //             .address = static_cast<flagged_address_t>(servo_address_t(static_cast<uint8_t>(command_t::WRITE_POS_EX), servo_id))},
    //         PacketManager::callback_config_t{
    //             .data_callback = [servo_idx](const Packet& packet)
    //             {
    //                 try
    //                 {
    //                     SimpleSerializable<write_pos_ex_t> cmd(packet.get_data());

    //                     int16_t position = cmd.position;
    //                     uint16_t speed = cmd.speed;
    //                     uint8_t acceleration = cmd.acceleration;

    //                     // servo.WritePosEx(servo_idx, position, speed, acceleration);
    //                     std::cout << "WRITE_POS_EX: servo=" << static_cast<int>(servo_idx)
    //                               << ", pos=" << position
    //                               << ", spd=" << speed
    //                               << ", acc=" << static_cast<int>(acceleration) << std::endl;
    //                 }
    //                 catch (...)
    //                 {
    //                     std::cout << "Error parsing WRITE_POS_EX" << std::endl;
    //                 }
    //             }});
    // }

    // packet_manager->set_callback(  // Register callback for PWM_1
    //     filter_t{
    //         .address = static_cast<flagged_address_t>(servo_address_t(static_cast<uint8_t>(command_t::SET_PWM_VALUES), motor_id_t::PWM_1))},
    //     PacketManager::callback_config_t{
    //         .data_callback = [](const Packet& packet)
    //         {
    //             try
    //             {
    //                 SimpleSerializable<set_pwm_raw_t> cmd(packet.get_data());
    //                 uint16_t pwm = cmd.pwm;
    //                 std::cout << "PWM1: value=" << pwm << std::endl;
    //             }
    //             catch (...)
    //             {
    //             }
    //         }});

    // packet_manager->set_callback(  // Register callback for PWM_2
    //     filter_t{
    //         .address = static_cast<flagged_address_t>(servo_address_t(static_cast<uint8_t>(command_t::SET_PWM_VALUES), motor_id_t::PWM_2))},
    //     PacketManager::callback_config_t{
    //         .data_callback = [](const Packet& packet)
    //         {
    //             try
    //             {
    //                 SimpleSerializable<set_pwm_raw_t> cmd(packet.get_data());
    //                 uint16_t pwm = cmd.pwm;
    //                 std::cout << "PWM2: value=" << pwm << std::endl;
    //             }
    //             catch (...)
    //             {
    //             }
    //         }});

    std::cout << "Setup complete - ready to receive CAN commands" << std::endl;
}

void loop() {
    static unsigned long last_debug = 0;

    try {
        packet_manager->handle();
    } catch (const std::exception& e) {
        std::cout << "CAN error: " << e.what() << std::endl;
    }
    // std::cout << "Handling CAN packets...4" << std::endl;

    // Debug: print status every 2 seconds
    last_debug++;
    if (/*millis() - */ last_debug > 2000000) {
        // last_debug = millis();
        // Serial.println("Waiting for CAN messages...");
        // std::cout << "Sending motor status messages..." << std::endl;
        // if (can_interface) {
        //     can_interface->transmit(Packet(static_cast<flagged_address_t>(
        //         standard_address_t{ base_address,
        //                            static_cast<uint8_t>(control_board::group::PWM_1),
        //                            static_cast<uint8_t>(control_board::pwm_parameters::SET_PWM) }),
        //         pwm_t{ 1234 }.serialize_data()));
        // }
        last_debug = 0;
    }

    // TODO: Control PWM output pin for PWM_1
}

int main() {
    setup();

    while (true) {
        loop();
    }
}

void handle_pwm_data(const Packet& packet) {
    try {
        // get target device group from address
        control_board::group group_id = static_cast<control_board::group>(
            static_cast<standard_address_t>(
                packet.get_address().address)
            .group);
        uint16_t pwm_value = pwm_t{ packet.get_data() }.value;
        // set pwm value
        if (group_id == control_board::group::PWM_1) {
            std::cout << std::format("Received PWM_1 command: {}\n", pwm_value);
        } else if (group_id == control_board::group::PWM_2) {
            std::cout << std::format("Received PWM_2 command: {}\n", pwm_value);
        }
    } catch (const std::exception& e) {
        std::cout << std::format("Failed to parse speed packet: {}\n", e.what());
    }
}

void register_pwm_device(const control_board::group& group) {
    const standard_address_t address{
        base_address,
        static_cast<uint8_t>(group),
        static_cast<uint8_t>(control_board::pwm_parameters::SET_PWM) };

    packet_manager->set_callback(
        filter_t{ static_cast<flagged_address_t>(address) },
        { .data_callback = handle_pwm_data });
}
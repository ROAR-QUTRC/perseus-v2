#include <chrono>
#include <hi_can_parameter.hpp>
#include <hi_can_raw.hpp>
#include <iostream>
#include <optional>
#include <thread>

using namespace std::chrono_literals;

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
void register_rsbl_servo(const control_board::group& group);
void handle_rsbl_servo_command(const Packet& packet);

const addressing::standard_address_t base_address{
    SYSTEM_ID,
    arm::SUBSYSTEM_ID,
    arm::control_board::DEVICE_ID};

void setup()
{
    can_interface.emplace(RawCanInterface("tyler"));
    packet_manager.emplace(can_interface.value());
    std::cout << "CAN interface initialized on vcan0" << std::endl;

    register_pwm_device(control_board::group::PWM_1);
    register_pwm_device(control_board::group::PWM_2);
    register_rsbl_servo(control_board::group::SHOULDER_TILT);
    register_rsbl_servo(control_board::group::SHOULDER_PAN);

    std::cout << "Setup complete - ready to receive CAN commands" << std::endl;
}

void loop()
{
    try
    {
        packet_manager->handle();
    }
    catch (const std::exception& e)
    {
        std::cout << "CAN error: " << e.what() << std::endl;
    }
}

int main()
{
    setup();

    while (true)
    {
        loop();
        std::this_thread::sleep_for(1ms);
    }
}

#pragma region RSBL Servo

void handle_rsbl_servo_command(const Packet& packet)
{
    using namespace control_board;

    try
    {
        // get target device group from address
        control_board::group group_id = static_cast<control_board::group>(
            static_cast<standard_address_t>(
                packet.get_address().address)
                .group);
        // Get the command type from the parameter ID
        control_board::rsbl_parameters command_type = static_cast<control_board::rsbl_parameters>(
            static_cast<standard_address_t>(
                packet.get_address().address)
                .parameter);

        std::cout << "Target device group: " << static_cast<int>(group_id) << std::endl;

        // action command
        switch (command_type)
        {
        case rsbl_parameters::SET_POS_EX:
        {
            const position_control_t position_control_cmd = position_control_t{packet.get_data()};
            std::cout << "SET_POS_EX command not implemented."
                      << " Target position: " << position_control_cmd.position
                      << ", speed: " << position_control_cmd.duration_ms
                      << ", acceleration: " << static_cast<int>(position_control_cmd.acceleration)
                      << std::endl;
            break;
        }
        case rsbl_parameters::SET_POSITION_SINGLE:
        {
            const int16_t position_cmd = position_t{packet.get_data()}.value;
            std::cout << "SET_POSITION_SINGLE command not implemented. Target position: " << position_cmd << std::endl;
            break;
        }
        case rsbl_parameters::SET_SPEED:
        {
            const uint16_t speed_cmd = speed_t{packet.get_data()}.value;
            std::cout << "SET_SPEED command not implemented. Target speed: " << speed_cmd << std::endl;
            break;
        }
        case rsbl_parameters::SET_TORQUE_ENABLE:
        {
            const bool torque_enable = torque_enable_t{packet.get_data()}.value;
            std::cout << "SET_TORQUE_ENABLE command not implemented. Target: " << torque_enable << std::endl;
            break;
        }
        case rsbl_parameters::STATUS_1:
        case rsbl_parameters::STATUS_2:
            // Not a command we handle
            break;
        default:
            std::cout << std::format("Unknown RSBL command type: {}\n", static_cast<uint8_t>(command_type));
            break;
        }
    }
    catch (const std::exception& e)
    {
        std::cout << std::format("Failed to parse RSBL servo packet: {}\n", e.what());
    }
}

void register_rsbl_servo(const control_board::group& group)
{
    standard_address_t address{
        base_address,
        static_cast<uint8_t>(group), 0};  // Parameter ignored by mask

    packet_manager->set_callback(
        filter_t{.address = static_cast<flagged_address_t>(address),
                 // Mask out parameter ID to catch all RSBL servo commands
                 .mask = 0xFFFFFF00},
        {.data_callback = handle_rsbl_servo_command});

    address.parameter = static_cast<uint8_t>(control_board::rsbl_parameters::STATUS_1);
    packet_manager->set_transmission_config(static_cast<flagged_address_t>(address),
                                            {.generator = [=]()
                                             {
                                                 status_1_t status{};  // TODO: get actual position
                                                 status.position = 12;
                                                 status.speed = 23;
                                                 status.load = 112;
                                                 return status.serialize_data();
                                             },
                                             .interval = 100ms});
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
            std::cout << std::format("Received PWM_1 command: {}\n", pwm_value);
        }
        else if (group_id == control_board::group::PWM_2)
        {
            std::cout << std::format("Received PWM_2 command: {}\n", pwm_value);
        }
    }
    catch (const std::exception& e)
    {
        std::cout << std::format("Failed to parse speed packet: {}\n", e.what());
    }
}

void register_pwm_device(const control_board::group& group)
{
    const standard_address_t address{
        base_address,
        static_cast<uint8_t>(group),
        static_cast<uint8_t>(control_board::pwm_parameters::SET_PWM)};

    packet_manager->set_callback(
        filter_t{static_cast<flagged_address_t>(address)},
        {.data_callback = handle_pwm_data});
}

#pragma endregion PWM Device
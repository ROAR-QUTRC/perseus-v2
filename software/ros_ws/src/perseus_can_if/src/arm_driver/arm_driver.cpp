#include "arm_driver/arm_driver.hpp"

#include <string>

// IF YOU'RE GOING TO IMPLEMENT SPEED CONTROL, ALSO IMPLEMENT A TIMEOUT TO STOP THE SERVO!!!

using namespace hi_can;
using namespace hi_can::addressing::post_landing::servo::rmd;
using namespace hi_can::parameters::post_landing::servo::rmd;

ArmDriver::ArmDriver(const rclcpp::NodeOptions& options)
    : Node("rmd_servo_driver", options)
{
    using namespace hi_can;
    try
    {
        _can_interface.emplace(RawCanInterface(this->declare_parameter("can_bus", "can0")));
        _packet_manager.emplace(_can_interface.value());
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialise CAN bus: %s", e.what());
        return;
    }

    _packet_manager->add_group(_ElbowParameterGroup);
    _packet_manager->add_group(_WristRollParameterGroup);
    _packet_manager->add_group(_WristYawParameterGroup);

    _can_interface->add_filter(_rmd_receive_filter);

    // Initialise all motors on startup with desired functions
    // Stop all motors - motors will lock up and won't flop around
    _can_interface->transmit(Packet{
        servo_address_t{rmd_id::MULTI_MOTOR_SEND},
        send_message::command_message_t(send_message::command_message_t::command_t::STOP).serialize_data()});
    // Set motors to transmit error messages when there is an error
    _can_interface->transmit(Packet{
        servo_address_t{rmd_id::MULTI_MOTOR_SEND},
        send_message::function_control_message_t(send_message::function_control_message_t::function_index_t::ERROR_TRANSMISSION_ENABLE, 1).serialize_data()});
    // Set motors to transmit all three status messages every second
    _can_interface->transmit(Packet{
        servo_address_t{rmd_id::MULTI_MOTOR_SEND},
        send_message::active_reply_message_t(send_message::active_reply_message_t::reply_t::STATUS_1, true, 1000).serialize_data()});
    _can_interface->transmit(Packet{
        servo_address_t{rmd_id::MULTI_MOTOR_SEND},
        send_message::active_reply_message_t(send_message::active_reply_message_t::reply_t::STATUS_2, true, 1000).serialize_data()});
    _can_interface->transmit(Packet{
        servo_address_t{rmd_id::MULTI_MOTOR_SEND},
        send_message::active_reply_message_t(send_message::active_reply_message_t::reply_t::STATUS_3, true, 1000).serialize_data()});

    // ROS
    _packet_timer = this->create_wall_timer(PACKET_HANDLE, std::bind(&ArmDriver::_can_handle, this));
    _command_subscriber = this->create_subscription<perseus_msgs::msg::ArmServoControl>("/arm/rmd_control", 10, std::bind(&ArmDriver::_position_control, this, std::placeholders::_1));
    _status_service = this->create_service<perseus_msgs::srv::RmdServoStatus>("/arm/rmd_status", std::bind(&ArmDriver::_get_rmd_status, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "RMD servo driver node initialised");
}

void ArmDriver::_position_control(perseus_msgs::msg::ArmServoControl servo_control)
{
    switch (servo_control.motor_id)
    {
    case perseus_msgs::msg::ArmServoControl::ELBOW:
        _can_interface->transmit(Packet(
            servo_address_t(rmd_id::SEND, motor_id_t::ELBOW),
            send_message::position_message_t(
                send_message::position_message_t::position_command_t::ABSOLUTE,
                RMD_SPEED_LIMIT,
                servo_control.absolute_position)
                .serialize_data()));
        break;
    case perseus_msgs::msg::ArmServoControl::WRIST_YAW:
        _can_interface->transmit(Packet(
            servo_address_t(rmd_id::SEND, motor_id_t::WRIST_YAW),
            send_message::position_message_t(
                send_message::position_message_t::position_command_t::ABSOLUTE,
                RMD_SPEED_LIMIT,
                servo_control.absolute_position)
                .serialize_data()));
        break;
    case perseus_msgs::msg::ArmServoControl::WRIST_ROLL:
        _can_interface->transmit(Packet(
            servo_address_t(rmd_id::SEND, motor_id_t::WRIST_ROLL),
            send_message::position_message_t(
                send_message::position_message_t::position_command_t::ABSOLUTE,
                RMD_SPEED_LIMIT,
                servo_control.absolute_position)
                .serialize_data()));
        break;
    case perseus_msgs::msg::ArmServoControl::SHOULDER_TILT:
        _can_interface->transmit(Packet(addressing::raw_address_t(addressing::standard_address_t(addressing::post_landing::SYSTEM_ID, addressing::post_landing::servo::SUBSYSTEM_ID, addressing::post_landing::servo::rsbl::DEVICE_ID, addressing::post_landing::servo::rsbl::rsbl_group::SHOULDER_TILT, addressing::post_landing::servo::rsbl::rsbl_function::SET_POSITION)),
                                        servo_control.absolute_position));
        break;
    case perseus_msgs::msg::ArmServoControl::SHOULDER_PAN:
        _can_interface->transmit(Packet(addressing::raw_address_t(addressing::standard_address_t(addressing::post_landing::SYSTEM_ID, addressing::post_landing::servo::SUBSYSTEM_ID, addressing::post_landing::servo::rsbl::DEVICE_ID, addressing::post_landing::servo::rsbl::rsbl_group::SHOULDER_PAN, addressing::post_landing::servo::rsbl::rsbl_function::SET_POSITION)),
                                        servo_control.absolute_position));
        break;
    default:
        break;
    }
}

void ArmDriver::_can_handle()
{
    try
    {
        _packet_manager->handle();
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, "CAN Handle failed: %s", e.what());
    }
    std::vector<std::string> elbow_errors = _ElbowParameterGroup.check_errors();
    std::vector<std::string> wrist_yaw_errors = _WristYawParameterGroup.check_errors();
    std::vector<std::string> wrist_roll_errors = _WristRollParameterGroup.check_errors();
    if (elbow_errors.size())
    {
        for (std::string error : elbow_errors)
        {
            RCLCPP_ERROR(this->get_logger(), "Elbow error: %s", error.c_str());
        }
    }
    if (wrist_yaw_errors.size())
    {
        for (std::string error : wrist_yaw_errors)
        {
            RCLCPP_ERROR(this->get_logger(), "Wrist yaw error: %s", error.c_str());
        }
    }
    if (wrist_roll_errors.size())
    {
        for (std::string error : wrist_roll_errors)
        {
            RCLCPP_ERROR(this->get_logger(), "Wrist roll error: %s", error.c_str());
        }
    }
}

void ArmDriver::_get_rmd_status(const std::shared_ptr<perseus_msgs::srv::RmdServoStatus::Request> request, std::shared_ptr<perseus_msgs::srv::RmdServoStatus::Response> response)
{
    RmdParameterGroup* servo_parameter_group;
    switch (static_cast<motor_id_t>(request->motor_id & 0x00F))
    {
    case motor_id_t::ELBOW:
        servo_parameter_group = &_ElbowParameterGroup;
        break;
    case motor_id_t::WRIST_ROLL:
        servo_parameter_group = &_WristRollParameterGroup;
        break;
    case motor_id_t::WRIST_YAW:
        servo_parameter_group = &_WristYawParameterGroup;
        break;
    default:
        return;
    }
    if (servo_parameter_group != nullptr)
    {
        response->temperature = servo_parameter_group->get_temp();
        response->brake_control = static_cast<bool>(servo_parameter_group->get_brake_status());
        response->voltage = servo_parameter_group->get_voltage();
        response->error = static_cast<uint16_t>(servo_parameter_group->get_error_status());
        response->torque_current = servo_parameter_group->get_torque_current();
        response->speed = servo_parameter_group->get_speed();
        response->angle = servo_parameter_group->get_angle();
        response->phase_a_current = servo_parameter_group->get_phase_a_current();
        response->phase_b_current = servo_parameter_group->get_phase_b_current();
        response->phase_c_current = servo_parameter_group->get_phase_c_current();
    }
}

void ArmDriver::cleanup()
{
    _packet_manager.reset();
    _can_interface.reset();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<ArmDriver>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting RMD Servo driver node");
        rclcpp::spin(node);
        node->cleanup();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running RMD Servo driver: %s", e.what());
        return EXIT_FAILURE;
    }
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
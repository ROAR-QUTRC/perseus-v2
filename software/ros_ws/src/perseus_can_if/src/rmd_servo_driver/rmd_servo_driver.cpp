#include "rmd_servo_driver/rmd_servo_driver.hpp"

using namespace hi_can;
using namespace hi_can::addressing::post_landing::servo::rmd;
using namespace hi_can::parameters::post_landing::servo::rmd;

RmdServoDriver::RmdServoDriver(const rclcpp::NodeOptions& options)
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
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<RmdServoDriver>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting RMD Servo driver node");
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running RMD Servo driver: %s", e.what());
        return EXIT_FAILURE;
    }
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
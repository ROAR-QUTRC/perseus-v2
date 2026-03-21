#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>

class LightController : public rclcpp::Node
{
public:
    explicit LightController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void _run_keyboard_loop();

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _command_publisher;
    std::thread _keyboard_thread;
};
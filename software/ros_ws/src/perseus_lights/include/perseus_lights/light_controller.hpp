#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte.hpp>

class LightController : public rclcpp::Node
{
public:
    explicit LightController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void _run_keyboard_loop();

    rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr _command_publisher;
    std::thread _keyboard_thread;
};
#pragma once

#include <chrono>
#include <hi_can_raw.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>

namespace ring
{
    enum class colours : uint32_t
    {
        WHITE = 0x00FFFFFF,
        RED = 0x00FF0000,
        BLUE = 0x000000FF,
        CYAN = 0x00FFFF00,
        GREEN = 0x0000FF00,
        YELLOW = 0x0000FFFF,
        MAGENTA = 0x00FF00FF,
    };
    enum class commands : uint8_t
    {
        WHITE = 0,
        RED = 1,
        BLUE = 2,
        CYAN = 3,
        GREEN = 4,
        YELLOW = 5,
        MAGENTA = 6,
    };
}

class LightDriver : public rclcpp::Node
{
public:
    explicit LightDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void _control_callback(const std_msgs::msg::Int8::SharedPtr msg);
    void _write_lights(ring::colours colour);

    hi_can::RawCanInterface _can_interface;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr _command_subscription;
};
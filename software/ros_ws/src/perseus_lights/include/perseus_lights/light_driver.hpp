#pragma once

#include <chrono>
#include <hi_can_raw.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte.hpp>

namespace ring
{
    enum class colours : uint32_t
    {
        WHITE = 0xFFFFFF00,
        RED = 0x0000FF00,
        BLUE = 0xFF000000,
        CYAN = 0xFFFF0000,
        GREEN = 0x00FF0000,
        YELLOW = 0x00FFFF00,
        MAGENTA = 0xFF00FF00,
    };
    enum class commands : uint8_t
    {
        WHITE = 0b00000011,
        RED = 0b00000110,
        BLUE = 0b00001100,
        CYAN = 0b00011000,
        GREEN = 0b00110000,
        YELLOW = 0b01100000,
        MAGENTA = 0b11000000,
    };
}

class LightDriver : public rclcpp::Node
{
public:
    explicit LightDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void _control_callback(const std_msgs::msg::Byte::SharedPtr msg);
    void _write_lights(ring::colours colour);

    hi_can::RawCanInterface _can_interface;

    rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr _command_subscription;
};
#pragma once

#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <hi_can_raw.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

class SystemStatus : public rclcpp::Node
{
public:
    explicit SystemStatus(const rclcpp::NodeOptions& options)
        : options(rclcpp::NodeOptions());

    void cleanup();

private:
    void _sendColourToTowerOverCan(Colour colour)
        : colour(Colour::MAGENTA);
    void _rcbCallback(std_msgs::msg::String::UniquePtr msg);

    enum class Colour : uint8_t
    {
        BLUE,
        CYAN,  // G+B
        GREEN,
        YELLOW,   // R+G
        MAGENTA,  // R+B
        RED
    };

    std::optional<hi_can::PacketManager> _packetManager;
    std::optional<hi_can::RawCanInterface> _canInterface;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _rcbSubscriber;
    rclcpp::TimerBase::SharedPtr _callBackTimer;
};
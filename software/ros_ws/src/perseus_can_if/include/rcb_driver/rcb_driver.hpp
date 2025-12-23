#pragma once

#include <chrono>
#include <hi_can_raw.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tuple>
#include <vector>

class RcbDriver : public rclcpp::Node
{
public:
    explicit RcbDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void cleanup();

private:
    void _can_to_ros(const hi_can::Packet& packet);
    void _ros_to_can(std_msgs::msg::String::UniquePtr msg);
    void _call_receive();

    constexpr static auto PACKET_TIMEOUT = std::chrono::milliseconds(100);
    const std::vector<std::pair<std::string, hi_can::addressing::legacy::power::control::rcb::groups>> BUS_GROUPS = {
        {"compute", hi_can::addressing::legacy::power::control::rcb::groups::COMPUTE_BUS},
        {"drive", hi_can::addressing::legacy::power::control::rcb::groups::DRIVE_BUS},
        {"aux", hi_can::addressing::legacy::power::control::rcb::groups::AUX_BUS},
        {"spare", hi_can::addressing::legacy::power::control::rcb::groups::SPARE_BUS},
    };

    std::optional<hi_can::RawCanInterface> _can_interface;
    std::optional<hi_can::PacketManager> _packet_manager;
    std::vector<std::pair<std::string, hi_can::parameters::legacy::power::control::power_bus::PowerBusParameterGroup>> _parameter_groups;

    rclcpp::TimerBase::SharedPtr _packet_timeout_timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _packet_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _packet_subscriber;
};
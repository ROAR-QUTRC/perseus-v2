#pragma once

#include <chrono>
#include <hi_can_raw.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tuple>

class RcbDriver : public rclcpp::Node
{
public:
    explicit RcbDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void cleanup();

private:
    void _canToRos();
    void _rosToCan(std_msgs::msg::String::UniquePtr msg);

    constexpr static auto PACKET_TIMEOUT = std::chrono::milliseconds(100);
    const std::vector<std::pair<std::string, hi_can::addressing::legacy::power::control::rcb::groups>> BUS_GROUPS = {
        {"compute", hi_can::addressing::legacy::power::control::rcb::groups::COMPUTE_BUS},
        {"drive", hi_can::addressing::legacy::power::control::rcb::groups::DRIVE_BUS},
        {"aux", hi_can::addressing::legacy::power::control::rcb::groups::AUX_BUS},
        {"spare", hi_can::addressing::legacy::power::control::rcb::groups::SPARE_BUS},
    };

    std::optional<hi_can::RawCanInterface> _canInterface;
    std::optional<hi_can::PacketManager> _packetManager;
    std::vector<std::pair<std::string, hi_can::parameters::legacy::power::control::power_bus::PowerBusParameterGroup>> _parameterGroups;

    rclcpp::TimerBase::SharedPtr _packetTimeoutTimer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _packetPublisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _packetSubscriber;
};
#pragma once

#include <chrono>
#include <hi_can_raw.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace hi_can::addressing::legacy::power::control::rcb;

class RcbDriver : public rclcpp::Node
{
public:
    explicit RcbDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void cleanup();

private:
    void _canToRos();
    void _rosToCan(std_msgs::msg::String::UniquePtr msg);

    constexpr static auto PACKET_TIMEOUT = std::chrono::milliseconds(100);
    const std::vector<groups> BUS_GROUPS = {
        groups::COMPUTE_BUS,
        groups::DRIVE_BUS,
        groups::AUX_BUS,
        groups::SPARE_BUS,
    };

    std::optional<hi_can::RawCanInterface> _canInterface;
    std::optional<hi_can::PacketManager> _packetManager;
    std::vector<hi_can::parameters::legacy::power::control::power_bus::PowerBusParameterGroup> _parameterGroups;

    rclcpp::TimerBase::SharedPtr _packetTimeoutTimer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _packetPublisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _packetSubscriber;
};
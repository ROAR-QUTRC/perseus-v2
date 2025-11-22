#pragma once

#include <chrono>
#include <hi_can_raw.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <string>

class SystemStatus : public rclcpp::Node
{
public:
    explicit SystemStatus(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void cleanup();

private:
    const hi_can::addressing::legacy::address_t _LIGHT_TOWER_ADDRESS = 0; // TODO: Replace with light tower address

    std::optional<hi_can::RawCanInterface> _canInterface;
    std::optional<hi_can::PacketManager> _packetManager;

    void _publishSystemStatusCallBack(void);
    void SystemStatus::_RcbCallback(std_msgs::msg::String msg);

    constexpr static auto CALLBACK_PERIOD_MS = std::chrono::milliseconds(100);

    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr _systemStatusPublisher;
    rclcpp::TimerBase::SharedPtr _callBackTimer;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _RcbSubscriber;
};
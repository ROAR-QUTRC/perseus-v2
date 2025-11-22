#include "system_status.hpp"

SystemStatus::SystemStatus(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("system_status", options)
{
    // initialise the CAN interface
    _canInterface.emplace(hi_can::RawCanInterface(this->declare_parameter("can_bus", "can0")));
    _packetManager.emplace(_canInterface.value());

    _callBackTimer = this->create_wall_timer(CALLBACK_PERIOD_MS, std::bind(&SystemStatus::_publishSystemStatusCallBack, this));
    _RcbSubscriber = this->create_subscription<std_msgs::msg::String>("can_to_ros", 10, std::bind(&SystemStatus::_RcbCallback, this, std::placeholders::_1));
}

void SystemStatus::_publishSystemStatusCallBack(void)
{
}

void SystemStatus::_RcbCallback(std_msgs::msg::String msg)
{
}

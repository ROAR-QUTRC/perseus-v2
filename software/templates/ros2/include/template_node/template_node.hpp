#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>

class TemplateNode : public rclcpp::Node
{
public:
    explicit TemplateNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void _templateCallback(const std_msgs::msg::UInt8 msg);
    void _timerCallback(void);

    constexpr static auto TIMER_PERIOD = std::chrono::milliseconds(100);

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr _templateSubscription;
    rclcpp::TimerBase::SharedPtr _templateTimer;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr _templatePublisher;

protected:
    uint8_t _lastMsgReceived;
};

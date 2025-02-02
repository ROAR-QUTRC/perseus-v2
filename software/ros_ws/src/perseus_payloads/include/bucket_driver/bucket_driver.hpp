#pragma once

#include <actuator_msgs/msg/actuators.hpp>
#include <chrono>
#include <hi_can_raw.hpp>
#include <rclcpp/rclcpp.hpp>

class BucketDriver : public rclcpp::Node
{
public:
    explicit BucketDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void _actuatorCallback(const actuator_msgs::msg::Actuators::SharedPtr msg);
    void _timeoutCallback();

    void _writeActuators(double lift, double tilt, double jaws, double rotate, bool magnet);

    constexpr static auto ACTUATOR_TIMEOUT = std::chrono::milliseconds(300);

    bool _timedOut = false;
    hi_can::RawCanInterface _canInterface;

    rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr _actuatorSubscription;
    rclcpp::TimerBase::SharedPtr _killTimer;
};
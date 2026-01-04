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
    void _actuator_callback(const actuator_msgs::msg::Actuators::SharedPtr msg);
    void _timeout_callback();

    void _write_actuators(double lift, double tilt, double jaws, double rotate, bool magnet);

    constexpr static auto ACTUATOR_TIMEOUT = std::chrono::milliseconds(300);

    bool _timed_out = false;
    hi_can::RawCanInterface _can_interface;

    rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr _actuator_subscription;
    rclcpp::TimerBase::SharedPtr _kill_timer;
};
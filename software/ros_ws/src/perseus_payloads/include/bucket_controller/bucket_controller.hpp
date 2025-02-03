#pragma once

#include <actuator_msgs/msg/actuators.hpp>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class BucketController : public rclcpp::Node
{
public:
    explicit BucketController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void _joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joySubscription;
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr _actuatorPublisher;
};

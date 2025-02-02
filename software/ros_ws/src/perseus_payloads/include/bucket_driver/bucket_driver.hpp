#pragma once

#include <actuator_msgs/msg/actuators.hpp>
#include <rclcpp/rclcpp.hpp>

class BucketDriver : public rclcpp::Node
{
public:
    explicit BucketDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
};
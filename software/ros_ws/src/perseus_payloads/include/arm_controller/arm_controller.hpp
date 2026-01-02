#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class ArmController : public rclcpp::Node
{
public:
    explicit ArmController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
};
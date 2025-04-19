#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class ArmDriver : public rclcpp::Node
{
public:
    explicit ArmDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
};
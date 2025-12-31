#pragma once
#include <rclcpp/rclcpp.hpp>

class ProcessingPlant : public rclcpp::Node
{
public:
    explicit ProcessingPlant(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
};
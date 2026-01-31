#pragma once

#include "rclcpp/rclcpp.hpp"

class SpaceResourcesController : public rclcpp::Node
{
public:
    SpaceResourcesController()
        : Node("space_resources_controller")
    {
        RCLCPP_INFO(this->get_logger(), "SpaceResourcesController node started");
        // publishers, subscribers, etc
    }
};

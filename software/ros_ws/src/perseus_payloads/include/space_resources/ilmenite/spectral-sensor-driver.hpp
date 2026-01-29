#pragma once

#include "rclcpp/rclcpp.hpp"

namespace space_resources
{

    class SpectralSensor : public rclcpp::Node
    {
    public:
        SpectralSensor()
            : Node("spectral_sensor")
        {
            RCLCPP_INFO(this->get_logger(), "SpectralSensor node started");
            // publishers, subscribers, etc
        }
    };

}
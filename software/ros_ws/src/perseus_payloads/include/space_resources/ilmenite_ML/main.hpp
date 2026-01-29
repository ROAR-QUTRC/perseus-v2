#pragma once

#include "rclcpp/rclcpp.hpp"

class IlmeniteML : public rclcpp::Node
{
public:
    IlmeniteML()
        : Node("ilmenite_ml")
    {
        RCLCPP_INFO(this->get_logger(), "IlmeniteML node started");
        // publishers, subscribers
    }
};

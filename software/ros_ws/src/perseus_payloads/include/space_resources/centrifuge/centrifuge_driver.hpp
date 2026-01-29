#pragma once

#include <rclcpp/rclcpp.hpp>

namespace space_resources
{

class CentrifugeDriver : public rclcpp::Node
{
public:
  explicit CentrifugeDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
// publishers, subscirbers, params

};

}
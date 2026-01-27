#pragma once

#include <rclcpp/rclcpp.hpp>

namespace space_resources
{

class IlmeniteSensor : public rclcpp::Node
{
public:
  explicit IlmeniteSensor();

private:
  //spectral sensor interface and led control
};

}

#pragma once

#include <rclcpp/rclcpp.hpp>

namespace space_resources
{

class IlmeniteMechanics : public rclcpp::Node
{
public:
  explicit IlmeniteMechanics();

private:
  // servos (tipping and lid), vibration motors and timing logic (e.g. each sample is vibrated for 20 seconds each) goes here
};

}

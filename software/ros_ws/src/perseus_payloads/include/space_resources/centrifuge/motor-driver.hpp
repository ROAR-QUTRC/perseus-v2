#pragma once

#include <actuator_msgs/msg/actuators_angular_velocity.hpp>
#include <rclcpp/rclcpp.hpp>

class CentrifugeDriver : public rclcpp::Node
{
public:
    explicit CentrifugeDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Publisher<actuator_msgs::msg::ActuatorsAngularVelocity>::SharedPtr _centrifuge_speed_publisher;
};

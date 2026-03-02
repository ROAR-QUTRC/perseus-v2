#pragma once

#include <rclcpp/rclcpp.hpp>
#include <actuator_msgs/msg/actuators_angular_velocity.hpp>


class CentrifugeDriver : public rclcpp::Node
{
public:
    explicit CentrifugeDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Publisher<actuator_msgs::msg::ActuatorsAngularVelocity>::SharedPtr _centrifuge_speed_publisher;
};

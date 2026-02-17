#pragma once

#include <actuator_msgs/msg/actuators_angular_velocity.hpp>
#include <perseus_interfaces/srv/concentration.hpp>
#include <perseus_interfaces/srv/illuminance_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class SpaceResourcesController : public rclcpp::Node
{
public:
    explicit SpaceResourcesController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void _speed_callback(const actuator_msgs::msg::ActuatorsAngularVelocity msg);
    rclcpp::Subscription<actuator_msgs::msg::ActuatorsAngularVelocity>::SharedPtr _centrifuge_speed_callback;
    rclcpp::Client<perseus_interfaces::srv::Concentration>::SharedPtr _water_concentration_service;
    rclcpp::Client<perseus_interfaces::srv::Concentration>::SharedPtr _ilmenite_concentration_service;
    rclcpp::Publisher<actuator_msgs::msg::ActuatorsAngularVelocity>::SharedPtr _centrifuge_control_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _centrifuge_brake_publisher;

    double _current_velocity;
    double _command_velocity;
};

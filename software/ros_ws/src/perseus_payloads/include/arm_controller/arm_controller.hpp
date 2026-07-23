#pragma once

// Unified arm-controller ROS bridge.
//
// Joint order:
// shoulder_pan, shoulder_tilt, elbow, wrist_pitch, wrist_roll, tool.
//
// Topics:
// /arm/control       - input Float64MultiArray containing six positions.
// /arm/status        - output unified driver status.
// /arm/positions     - output six current positions.
// /arm/can/control   - output Actuators command for the CAN driver.
// /arm/can/status    - input status from the CAN driver.
// /arm/can/positions - input positions from the CAN driver.

#include <actuator_msgs/msg/actuators.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <cstddef>

class ArmController : public rclcpp::Node
{
public:
    explicit ArmController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void cleanup();

private:
    void _handle_arm_control(
        const std_msgs::msg::Float64MultiArray::SharedPtr message);
    void _handle_driver_status(
        const std_msgs::msg::Float64MultiArray::SharedPtr message);
    void _handle_driver_positions(
        const std_msgs::msg::Float64MultiArray::SharedPtr message);

    static constexpr std::size_t JOINT_COUNT = 6;

    double _default_velocity = 0.5;
    double _default_acceleration = 0.5;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
        _arm_control_subscriber;
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr
        _driver_control_publisher;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
        _driver_status_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
        _driver_positions_subscriber;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        _arm_status_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        _arm_positions_publisher;
};

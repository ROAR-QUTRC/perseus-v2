#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "st3215_servo/st3215.hpp"  // Include the servo library

class ArmDriver : public rclcpp::Node
{
public:
    explicit ArmDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // Timer for controlling the sweep motion
    rclcpp::TimerBase::SharedPtr _sweep_timer;

    // Servo instance
    std::unique_ptr<ST3215> _servo;

    // Sweep control variables
    bool _sweep_direction{true};  // true = increasing position
    static constexpr uint16_t _min_pos{0};
    static constexpr uint16_t _max_pos{1023};
    static constexpr uint16_t _step_size{10};
    uint16_t _current_pos{_min_pos};

    // Timer callback for sweeping motion
    void _sweep_callback();

    // Joint state publisher
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _joint_state_pub;

    // Publish the current joint state
    void _publish_joint_state(uint16_t position);
};
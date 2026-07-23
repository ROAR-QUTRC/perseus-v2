#include "arm_controller/arm_controller.hpp"

// Arm status messages are a 1D array of 13-element joint groups:
// [
//   joint_id,
//   has_responded,
//   ready,
//   enabled,
//   zeroed,
//   fault_code,
//   position,
//   speed,
//   load,
//   voltage,
//   temperature,
//   current,
//   moving,
// ]

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

ArmController::ArmController(const rclcpp::NodeOptions& options)
    : Node("arm_controller", options)
{
    _default_velocity =
        declare_parameter<double>("default_velocity_rad_s", 0.5);
    _default_acceleration =
        declare_parameter<double>("default_acceleration", 0.5);

    if (!std::isfinite(_default_velocity) || _default_velocity < 0.0)
        throw std::invalid_argument("default_velocity_rad_s must be non-negative");
    if (!std::isfinite(_default_acceleration))
        throw std::invalid_argument("default_acceleration must be finite");
    _default_acceleration = std::clamp(_default_acceleration, 0.0, 1.0);

    const auto control_topic =
        declare_parameter<std::string>("control_topic", "/arm/control");
    const auto status_topic =
        declare_parameter<std::string>("status_topic", "/arm/status");
    const auto positions_topic =
        declare_parameter<std::string>("positions_topic", "/arm/positions");
    const auto driver_control_topic =
        declare_parameter<std::string>("driver_control_topic", "/arm/can/control");
    const auto driver_status_topic =
        declare_parameter<std::string>("driver_status_topic", "/arm/can/status");
    const auto driver_positions_topic =
        declare_parameter<std::string>("driver_positions_topic", "/arm/can/positions");

    _driver_control_publisher =
        create_publisher<actuator_msgs::msg::Actuators>(driver_control_topic, 10);
    _arm_status_publisher =
        create_publisher<std_msgs::msg::Float64MultiArray>(status_topic, 10);
    _arm_positions_publisher =
        create_publisher<std_msgs::msg::Float64MultiArray>(positions_topic, 10);

    _arm_control_subscriber =
        create_subscription<std_msgs::msg::Float64MultiArray>(
            control_topic,
            10,
            std::bind(
                &ArmController::_handle_arm_control,
                this,
                std::placeholders::_1));
    _driver_status_subscriber =
        create_subscription<std_msgs::msg::Float64MultiArray>(
            driver_status_topic,
            10,
            std::bind(
                &ArmController::_handle_driver_status,
                this,
                std::placeholders::_1));
    _driver_positions_subscriber =
        create_subscription<std_msgs::msg::Float64MultiArray>(
            driver_positions_topic,
            10,
            std::bind(
                &ArmController::_handle_driver_positions,
                this,
                std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Arm controller ready for %zu joints",
        JOINT_COUNT);
}

void ArmController::_handle_arm_control(
    const std_msgs::msg::Float64MultiArray::SharedPtr message)
{
    if (message->data.size() != JOINT_COUNT)
    {
        RCLCPP_WARN(
            get_logger(),
            "Rejected arm command: expected %zu positions, received %zu",
            JOINT_COUNT,
            message->data.size());
        return;
    }

    if (!std::all_of(
            message->data.begin(),
            message->data.end(),
            [](double position) { return std::isfinite(position); }))
    {
        RCLCPP_WARN(get_logger(), "Rejected arm command containing a non-finite position");
        return;
    }

    actuator_msgs::msg::Actuators command;
    command.header.stamp = now();
    command.position = message->data;
    command.velocity.assign(JOINT_COUNT, _default_velocity);
    command.normalized.assign(JOINT_COUNT, _default_acceleration);
    _driver_control_publisher->publish(command);
}

void ArmController::_handle_driver_status(
    const std_msgs::msg::Float64MultiArray::SharedPtr message)
{
    _arm_status_publisher->publish(*message);
}

void ArmController::_handle_driver_positions(
    const std_msgs::msg::Float64MultiArray::SharedPtr message)
{
    if (message->data.size() != JOINT_COUNT)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Rejected driver positions: expected %zu values, received %zu",
            JOINT_COUNT,
            message->data.size());
        return;
    }

    _arm_positions_publisher->publish(*message);
}

void ArmController::cleanup()
{
    RCLCPP_INFO(get_logger(), "Arm controller stopped");
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<ArmController>();
        rclcpp::spin(node);
        node->cleanup();
    }
    catch (const std::exception& error)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("arm_controller"),
            "Arm controller failed: %s",
            error.what());
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}

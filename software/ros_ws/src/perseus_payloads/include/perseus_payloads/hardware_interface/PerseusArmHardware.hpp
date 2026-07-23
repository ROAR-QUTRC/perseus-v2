#pragma once

// ros2_control system interface for the unified six-joint arm.
//
// Driver order:
// shoulder_pan, shoulder_tilt, elbow, wrist_pitch, wrist_roll, fake_dof/tool.
//
// The interface publishes position/velocity commands to /arm/can/control and
// receives measured joint positions from /arm/can/positions.

#include <actuator_msgs/msg/actuators.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <array>
#include <cstddef>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

namespace perseus_payloads
{

class PerseusArmHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(PerseusArmHardware)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& parameters) override;
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;
    hardware_interface::return_type write(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;

private:
    static constexpr std::size_t JOINT_COUNT = 6;
    static constexpr std::array<std::string_view, JOINT_COUNT> JOINT_ORDER{
        "shoulder_pan",
        "shoulder_tilt",
        "elbow",
        "wrist_pitch",
        "wrist_roll",
        "fake_dof",
    };

    bool _validate_hardware_info() const;
    bool _cache_interface_handles();
    void _position_callback(
        const std_msgs::msg::Float64MultiArray::SharedPtr message);
    std::string _hardware_parameter(
        std::string_view name,
        std::string default_value) const;

    std::array<hardware_interface::StateInterface::SharedPtr, JOINT_COUNT>
        _position_state_interfaces{};
    std::array<hardware_interface::StateInterface::SharedPtr, JOINT_COUNT>
        _velocity_state_interfaces{};
    std::array<hardware_interface::CommandInterface::SharedPtr, JOINT_COUNT>
        _position_command_interfaces{};
    std::array<hardware_interface::CommandInterface::SharedPtr, JOINT_COUNT>
        _velocity_command_interfaces{};

    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr _command_publisher;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
        _position_subscriber;

    mutable std::mutex _feedback_mutex;
    std::array<double, JOINT_COUNT> _latest_positions{};
    bool _feedback_received = false;
    bool _new_feedback = false;
    bool _active = false;
    double _default_acceleration = 0.5;
};

}  // namespace perseus_payloads

#pragma once

#include <chrono>
#include <hi_can_raw.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "perseus_msgs/msg/arm_control.hpp"

// Arm CAN Controller - handles RSBL servos, PWM servos via CAN
//
// Topics:
// /arm
//     /rsbl
//          /control (subscriber) - ArmControl message with position, velocity, normalized
//          /status (publisher) - Status messages from all servos
//          /positions (publisher) - Current positions of all servos

class ArmController : public rclcpp::Node
{
public:
    explicit ArmController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void cleanup();

private:
    void _handle_arm_control(const perseus_msgs::msg::ArmControl::SharedPtr msg);
    void _publish_status_messages();
    void _publish_motor_positions();
    void _handle_can();
    void _request_servo_status();

    constexpr static auto PACKET_DELAY_MS = std::chrono::milliseconds(20);

    // CAN handling
    constexpr static auto PACKET_HANDLE_MS = std::chrono::milliseconds(100);
    rclcpp::TimerBase::SharedPtr _packet_timer;
    std::optional<hi_can::RawCanInterface> _can_interface;
    std::optional<hi_can::PacketManager> _packet_manager;

    // Servo board parameter groups
    const std::unordered_map<hi_can::addressing::post_landing::arm::control_board::group,
                             std::shared_ptr<hi_can::parameters::post_landing::arm::control_board::ControlBoardParameterGroup>>
        PARAMETER_GROUP_MAP = {
            // {hi_can::addressing::post_landing::arm::control_board::group::SHOULDER_TILT,
            //  std::make_shared<hi_can::parameters::post_landing::arm::control_board::ControlBoardParameterGroup>(
            //      static_cast<uint8_t>(hi_can::addressing::post_landing::arm::control_board::group::SHOULDER_TILT))},
            // {hi_can::addressing::post_landing::arm::control_board::group::SHOULDER_PAN,
            //  std::make_shared<hi_can::parameters::post_landing::arm::control_board::ControlBoardParameterGroup>(
            //      static_cast<uint8_t>(hi_can::addressing::post_landing::arm::control_board::group::SHOULDER_PAN))},
        };

    // Motor feedback
    constexpr static auto POSITION_PUBLISH_MS = std::chrono::milliseconds(100);
    constexpr static auto STATUS_REQUEST_MS = std::chrono::milliseconds(500);
    std::vector<hi_can::addressing::post_landing::arm::control_board::group> _available_servos;
    uint16_t _status_message_ms = 500;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _status_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _motor_position_publisher;
    rclcpp::TimerBase::SharedPtr _status_timer;
    rclcpp::TimerBase::SharedPtr _motor_position_timer;
    rclcpp::TimerBase::SharedPtr _status_request_timer;

    // Subscriber
    rclcpp::Subscription<perseus_msgs::msg::ArmControl>::SharedPtr _arm_control_subscriber;
};

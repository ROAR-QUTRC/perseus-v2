#pragma once

#include <actuator_msgs/msg/actuators.hpp>
#include <chrono>
#include <hi_can_raw.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "perseus_interfaces/srv/request_int8_array.hpp"

// Arm CAN Controller - handles RSBL servos, PWM servos via CAN
//
// Topics:
// /arm
//     /rsbl
//          /control (subscriber) - ArmControl message with position, velocity, normalized
//          /status (publisher) - Status messages from all servos
//          /positions (publisher) - Current positions of all servos

class RsblDriver : public rclcpp::Node
{
public:
    explicit RsblDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void cleanup();

private:
    void _handle_arm_control(const actuator_msgs::msg::Actuators::SharedPtr msg);
    void _publish_status_messages();
    void _publish_motor_positions();
    void _handle_can();

    constexpr static auto PACKET_DELAY_MS = std::chrono::milliseconds(20);
    const hi_can::addressing::standard_address_t baseAddress{
        hi_can::addressing::post_landing::SYSTEM_ID,
        hi_can::addressing::post_landing::arm::SUBSYSTEM_ID,
        hi_can::addressing::post_landing::arm::control_board::DEVICE_ID};

    // CAN handling
    constexpr static auto PACKET_HANDLE_MS = std::chrono::milliseconds(100);
    rclcpp::TimerBase::SharedPtr _packet_timer;
    std::optional<hi_can::RawCanInterface> _can_interface;
    std::optional<hi_can::PacketManager> _packet_manager;

    // Servo board parameter groups
    const std::unordered_map<hi_can::addressing::post_landing::arm::control_board::rsbl_group,
                             std::shared_ptr<hi_can::parameters::post_landing::arm::control_board::rsblParameterGroup>>
        PARAMETER_GROUP_MAP = {
            {hi_can::addressing::post_landing::arm::control_board::rsbl_group::ELBOW,
             std::make_shared<hi_can::parameters::post_landing::arm::control_board::rsblParameterGroup>(
                 hi_can::addressing::post_landing::arm::control_board::rsbl_group::ELBOW)},
            {hi_can::addressing::post_landing::arm::control_board::rsbl_group::SHOULDER_PAN,
             std::make_shared<hi_can::parameters::post_landing::arm::control_board::rsblParameterGroup>(
                 hi_can::addressing::post_landing::arm::control_board::rsbl_group::SHOULDER_PAN)},
            {hi_can::addressing::post_landing::arm::control_board::rsbl_group::SHOULDER_TILT,
             std::make_shared<hi_can::parameters::post_landing::arm::control_board::rsblParameterGroup>(
                 hi_can::addressing::post_landing::arm::control_board::rsbl_group::SHOULDER_TILT)},
        };

    // Motor feedback
    constexpr static auto POSITION_PUBLISH_MS = std::chrono::milliseconds(100);
    constexpr static auto STATUS_REQUEST_MS = std::chrono::milliseconds(500);
    std::vector<hi_can::addressing::post_landing::arm::control_board::rsbl_group> _available_servos;
    uint16_t _status_message_ms = 500;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _status_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _motor_position_publisher;
    rclcpp::TimerBase::SharedPtr _status_timer;
    rclcpp::TimerBase::SharedPtr _motor_position_timer;

    // Subscriber
    rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr _arm_control_subscriber;
};

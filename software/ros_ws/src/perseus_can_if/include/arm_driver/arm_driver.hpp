#include <chrono>
#include <hi_can_raw.hpp>
#include <rclcpp/rclcpp.hpp>

#include "perseus_msgs/msg/arm_servo_control.hpp"
#include "perseus_msgs/srv/rmd_servo_status.hpp"

class ArmDriver : public rclcpp::Node
{
public:
    explicit ArmDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void cleanup();

private:
    void _can_handle();
    void _position_control(perseus_msgs::msg::ArmServoControl servo_control);
    void _get_rmd_status(const std::shared_ptr<perseus_msgs::srv::RmdServoStatus::Request> request, std::shared_ptr<perseus_msgs::srv::RmdServoStatus::Response> response);

    std::optional<hi_can::RawCanInterface> _can_interface;
    std::optional<hi_can::PacketManager> _packet_manager;

    constexpr static auto PACKET_HANDLE = std::chrono::milliseconds(100);
    constexpr static uint16_t RMD_SPEED_LIMIT = 1000;

    hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup _ElbowParameterGroup = hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup(static_cast<uint8_t>(hi_can::addressing::post_landing::servo::rmd::motor_id_t::ELBOW));
    hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup _WristRollParameterGroup = hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup(static_cast<uint8_t>(hi_can::addressing::post_landing::servo::rmd::motor_id_t::WRIST_ROLL));
    hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup _WristYawParameterGroup = hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup(static_cast<uint8_t>(hi_can::addressing::post_landing::servo::rmd::motor_id_t::WRIST_YAW));

    hi_can::addressing::filter_t _rmd_receive_filter = {
        .address = hi_can::addressing::post_landing::servo::rmd::servo_address_t(hi_can::addressing::post_landing::servo::rmd::rmd_id::RECEIVE),
        .mask = 0xFF0,
    };
    hi_can::addressing::filter_t _rsbl_receive_filter = {
        .address = hi_can::addressing::raw_address_t(hi_can::addressing::standard_address_t(hi_can::addressing::post_landing::SYSTEM_ID, hi_can::addressing::post_landing::servo::SUBSYSTEM_ID, hi_can::addressing::post_landing::servo::rsbl::DEVICE_ID, 0x00, 0x00)),
        .mask = hi_can::addressing::DEVICE_MASK,
    };

    rclcpp::TimerBase::SharedPtr _packet_timer;
    rclcpp::Subscription<perseus_msgs::msg::ArmServoControl>::SharedPtr _command_subscriber;
    rclcpp::Service<perseus_msgs::srv::RmdServoStatus>::SharedPtr _status_service;
};
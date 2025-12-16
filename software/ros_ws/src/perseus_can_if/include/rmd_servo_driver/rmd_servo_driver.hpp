#include <chrono>
#include <hi_can_raw.hpp>
#include <rclcpp/rclcpp.hpp>

#include "perseus_msgs/msg/rmd_servo_status.hpp"

class RmdServoDriver : public rclcpp::Node
{
public:
    explicit RmdServoDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void _can_handle();

    std::optional<hi_can::RawCanInterface> _can_interface;
    std::optional<hi_can::PacketManager> _packet_manager;

    constexpr static auto PACKET_HANDLE = std::chrono::milliseconds(100);

    hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup _ElbowParameterGroup = hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup(hi_can::addressing::post_landing::servo::rmd::motor_id::ELBOW);
    hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup _WristRollParameterGroup = hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup(hi_can::addressing::post_landing::servo::rmd::motor_id::WRIST_ROLL);
    hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup _WristYawParameterGroup = hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup(hi_can::addressing::post_landing::servo::rmd::motor_id::WRIST_YAW);

    hi_can::addressing::filter_t _rmd_receive_filter = {
        .address = hi_can::addressing::post_landing::servo::rmd::servo_address_t(hi_can::addressing::post_landing::servo::rmd::rmd_id::RECEIVE),
        .mask = 0xFF0,
    };

    rclcpp::TimerBase::SharedPtr _packet_timer;
    rclcpp::Subscription<perseus_msgs::msg::rmd_control>::SharedPtr _position_control;
};
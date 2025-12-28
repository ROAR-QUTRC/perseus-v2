#include <chrono>
#include <hi_can_raw.hpp>
#include <rclcpp/rclcpp.hpp>

// #include "perseus_msgs/msg/arm_servo_control.hpp"
#include <map>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "perseus_msgs/msg/arm_config.hpp"
#include "perseus_msgs/msg/arm_servo_control.hpp"
#include "perseus_msgs/srv/rmd_brake.hpp"
#include "perseus_msgs/srv/rmd_can_id.hpp"
#include "perseus_msgs/srv/rmd_data.hpp"
#include "perseus_msgs/srv/rmd_servo_status.hpp"

class ArmDriver : public rclcpp::Node
{
public:
    explicit ArmDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void cleanup();

private:
    void _enable_status_messages(bool enable);
    void _can_handle();
    void _handle_rmd_message(const hi_can::Packet& packet);
    void _position_control(std_msgs::msg::Float64MultiArray servo_control);
    void _get_rmd_status(const std::shared_ptr<perseus_msgs::srv::RmdServoStatus::Request> request, std::shared_ptr<perseus_msgs::srv::RmdServoStatus::Response> response);
    void _get_rmd_can_ids(const std::shared_ptr<perseus_msgs::srv::RmdCanId::Request> request, std::shared_ptr<perseus_msgs::srv::RmdCanId::Response> response);
    void _set_brake_enabled(const std::shared_ptr<perseus_msgs::srv::RmdBrake::Request> request, std::shared_ptr<perseus_msgs::srv::RmdBrake::Response> response);
    void _restart_motor(const std::shared_ptr<perseus_msgs::srv::RmdData::Request> request, std::shared_ptr<perseus_msgs::srv::RmdData::Response> response);
    void _set_motor_id(const std::shared_ptr<perseus_msgs::srv::RmdData::Request> request, std::shared_ptr<perseus_msgs::srv::RmdData::Response> response);

    // std::map<hi_can::addressing::post_landing::servo::rmd::motor_id_t, rmd_status_t> _rmd_status_map;
    std::vector<hi_can::addressing::post_landing::servo::rmd::motor_id_t> _available_servos;

    std::optional<hi_can::RawCanInterface> _can_interface;
    std::optional<hi_can::PacketManager> _packet_manager;

    constexpr static auto PACKET_HANDLE = std::chrono::milliseconds(100);
    constexpr static uint16_t RMD_SPEED_LIMIT = 1000;
    constexpr static uint16_t STATUS_MESSAGE_MS = 200;
    // constexpr static uint16_t STATUS_MESSAGE_MS = 1500;
    constexpr static uint8_t PACKET_DELAY_MS = 20;

    bool _status_messages_enabled = false;

    // parameter group map
    const std::map<hi_can::addressing::post_landing::servo::rmd::motor_id_t, hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup> _parameter_group_map = {
        {hi_can::addressing::post_landing::servo::rmd::motor_id_t::ELBOW, hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup(static_cast<uint8_t>(hi_can::addressing::post_landing::servo::rmd::motor_id_t::ELBOW))},
        {hi_can::addressing::post_landing::servo::rmd::motor_id_t::WRIST_YAW, hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup(static_cast<uint8_t>(hi_can::addressing::post_landing::servo::rmd::motor_id_t::WRIST_YAW))},
        {hi_can::addressing::post_landing::servo::rmd::motor_id_t::WRIST_ROLL, hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup(static_cast<uint8_t>(hi_can::addressing::post_landing::servo::rmd::motor_id_t::WRIST_ROLL))},
    };

    hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup _ElbowParameterGroup = hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup(static_cast<uint8_t>(hi_can::addressing::post_landing::servo::rmd::motor_id_t::ELBOW));
    hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup _WristRollParameterGroup = hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup(static_cast<uint8_t>(hi_can::addressing::post_landing::servo::rmd::motor_id_t::WRIST_ROLL));
    hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup _WristYawParameterGroup = hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup(static_cast<uint8_t>(hi_can::addressing::post_landing::servo::rmd::motor_id_t::WRIST_YAW));

    rclcpp::TimerBase::SharedPtr _packet_timer;
    rclcpp::TimerBase::SharedPtr _check_available_servos_timer;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr _command_subscriber;
    rclcpp::Service<perseus_msgs::srv::RmdServoStatus>::SharedPtr _status_service;
    rclcpp::Service<perseus_msgs::srv::RmdCanId>::SharedPtr _can_id_service;
    rclcpp::Service<perseus_msgs::srv::RmdBrake>::SharedPtr _enable_brake_service;
    rclcpp::Service<perseus_msgs::srv::RmdData>::SharedPtr _restart_motor_service;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _enable_debug_service;
    rclcpp::Service<perseus_msgs::srv::RmdData>::SharedPtr _set_motor_id_service;
};
// #include <actuator_msgs/msg/actuators.hpp>
#include <chrono>
#include <hi_can_raw.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "perseus_msgs/srv/request_int8_array.hpp"
#include "perseus_msgs/srv/trigger_device.hpp"

class RmdDriver : public rclcpp::Node
{
public:
    explicit RmdDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void cleanup();

private:
    void _enable_status_messages(bool debug_mode);
    void _handle_position_control(std_msgs::msg::Float64MultiArray servo_control);
    void _publish_status_messages();
    void _publish_motor_positions();
    void _handle_can();
    std::vector<hi_can::addressing::post_landing::servo::rmd::motor_id_t> _get_online_servos();
    void _set_motor_id(
        const std::shared_ptr<perseus_msgs::srv::TriggerDevice::Request> request,
        std::shared_ptr<perseus_msgs::srv::TriggerDevice::Response> response);
    void _set_zero_position(
        const std::shared_ptr<perseus_msgs::srv::TriggerDevice::Request> request,
        std::shared_ptr<perseus_msgs::srv::TriggerDevice::Response> response);
    void _restart_motor(
        const std::shared_ptr<perseus_msgs::srv::TriggerDevice::Request> request,
        std::shared_ptr<perseus_msgs::srv::TriggerDevice::Response> response);
    void _get_rmd_can_ids(
        const std::shared_ptr<perseus_msgs::srv::RequestInt8Array::Request> request,
        std::shared_ptr<perseus_msgs::srv::RequestInt8Array::Response> response);

    constexpr static uint16_t RMD_SPEED_LIMIT = UINT16_MAX;  // max speed
    constexpr static auto PACKET_DELAY_MS = std::chrono::milliseconds(25);
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr _motor_target_positions_subscriber;

    // CAN handling
    constexpr static auto PACKET_HANDLE_MS = std::chrono::milliseconds(50);
    rclcpp::TimerBase::SharedPtr _packet_timer;
    std::optional<hi_can::RawCanInterface> _can_interface;
    std::optional<hi_can::PacketManager> _packet_manager;
    const std::unordered_map<hi_can::addressing::post_landing::servo::rmd::motor_id_t, std::shared_ptr<hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup>> PARAMETER_GROUP_MAP = {
        {hi_can::addressing::post_landing::servo::rmd::motor_id_t::ELBOW,
         std::make_shared<hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup>(static_cast<uint8_t>(hi_can::addressing::post_landing::servo::rmd::motor_id_t::ELBOW))},
        {hi_can::addressing::post_landing::servo::rmd::motor_id_t::WRIST_TILT,
         std::make_shared<hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup>(static_cast<uint8_t>(hi_can::addressing::post_landing::servo::rmd::motor_id_t::WRIST_TILT))},
        {hi_can::addressing::post_landing::servo::rmd::motor_id_t::WRIST_PAN,
         std::make_shared<hi_can::parameters::post_landing::servo::rmd::RmdParameterGroup>(static_cast<uint8_t>(hi_can::addressing::post_landing::servo::rmd::motor_id_t::WRIST_PAN))},
    };

    // Motor feedback
    uint16_t _status_message_ms = 500;  // Default to 500ms when not in debug mode
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _status_publisher;
    rclcpp::TimerBase::SharedPtr _status_timer;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _enable_debug_stats_service;

    // Config Services
    rclcpp::Service<perseus_msgs::srv::TriggerDevice>::SharedPtr _set_motor_id_service;
    rclcpp::Service<perseus_msgs::srv::TriggerDevice>::SharedPtr _set_zero_position_service;
    rclcpp::Service<perseus_msgs::srv::RequestInt8Array>::SharedPtr _get_can_ids_service;
    rclcpp::Service<perseus_msgs::srv::TriggerDevice>::SharedPtr _restart_motor_service;
};
#include <hi_can_raw.hpp>
#include <rclcpp/rclcpp.hpp>

#include "perseus_msgs/msg/rmd_servo_status.hpp"

class RmdServoDriver : public rclcpp::Node
{
public:
    explicit RmdServoDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    std::optional<hi_can::RawCanInterface> _can_interface;
    std::optional<hi_can::PacketManager> _packet_manager;

    hi_can::addressing::filter_t _rmd_receive_filter = {
        .address = hi_can::addressing::post_landing::servo::rmd::servo_address_t(hi_can::addressing::post_landing::servo::rmd::rmd_command::RECEIVE),
        .mask = 0xFF0,
    };
    const hi_can::parameters::post_landing::servo::rmd::send_message::command_message_t STOP_MESSAGE(
        hi_can::parameters::post_landing::servo::rmd::send_message::command_message_t::command_t::STOP);

    rclcpp::Publisher<perseus_msgs::msg::RmdServoStatus>::SharedPtr _status_publisher;
};
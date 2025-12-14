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

    rclcpp::Publisher<perseus_msgs::msg::RmdServoStatus>::SharedPtr _status_publisher;
};
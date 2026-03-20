#include <chrono>
#include <hi_can_raw.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

// Topics:
// /arm
//     /end_effector
//          /read (publisher uint16MultiArray) - Current state of the end effector
//          /write (subscriber uint16MultiArray) - Control commands for the end effector

class EndEffector : public rclcpp::Node
{
public:
    explicit EndEffector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void cleanup();

private:
    // translate ros to can to write an analog value
    void _handle_write(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
    // translate can to ros to publish the current state of the end effector
    void _handle_read();
    void _handle_can_packets();

    constexpr static auto PUBLISH_TIMER_MS = std::chrono::milliseconds(100);
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr _read;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr _write;

    // CAN handling
    constexpr static auto PACKET_HANDLE_MS = std::chrono::milliseconds(50);
    rclcpp::TimerBase::SharedPtr _packet_timer;
    std::optional<hi_can::RawCanInterface> _can_interface;
    std::optional<hi_can::PacketManager> _packet_manager;
    const std::shared_ptr<hi_can::parameters::post_landing::arm::control_board::pwmParameterGroup> _pwm_parameter_group = 
    std::make_shared<hi_can::parameters::post_landing::arm::control_board::pwmParameterGroup>();
};
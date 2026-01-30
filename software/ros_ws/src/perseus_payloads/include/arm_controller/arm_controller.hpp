#include <actuator_msgs/msg/actuators.hpp>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// Topics:
// /arm
//     /rsbl
//          /control (publisher Actuators) - ArmControl message with position, velocity, normalized
//          /status (subscriber Float64MultiArray) - Status messages from all servos
//          /positions (subscriber Float64MultiArray) - Current positions of all servos
//     /rmd
//         /control (publisher Actuators) - target positions for servos
//         /status (subscriber Float64MultiArray) - status messages from servos
//     /control (subscriber Float64MultiArray) - High level control parameters for the arm
//     /position (publisher Float64MultiArray) - High level control parameters for the arm

class ArmController : public rclcpp::Node
{
public:
    explicit ArmController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void cleanup();

private:
    void _handle_arm_control(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void _receive_rmd_positions(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void _receive_rmd_status(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void _receive_rsbl_positions(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void _receive_rsbl_status(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void _publish_arm_status();
    void _publish_arm_position();

    constexpr static auto PUBLISH_TIMER_MS = std::chrono::milliseconds(10);
    std::vector<double> _current_arm_positions;
    rclcpp::TimerBase::SharedPtr _position_timer;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _arm_status_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _arm_position_publisher;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr _arm_control_subscriber;

    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr _rmd_control_publisher;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr _rmd_status_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr _rmd_position_subscriber;

    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr _rsbl_control_publisher;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr _rsbl_status_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr _rsbl_position_subscriber;
};
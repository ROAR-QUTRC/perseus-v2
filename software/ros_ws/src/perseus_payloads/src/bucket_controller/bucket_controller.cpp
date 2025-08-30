#include "bucket_controller/bucket_controller.hpp"

#include <algorithm>
#include <cstdint>

BucketController::BucketController(const rclcpp::NodeOptions& options)
    : Node("bucket_controller", options)
{
    _joy_subscription =
        this->create_subscription<sensor_msgs::msg::Joy>("bucket_joy", 10, std::bind(&BucketController::_joy_callback, this, std::placeholders::_1));
    _actuator_publisher = this->create_publisher<actuator_msgs::msg::Actuators>("bucket_actuators", 10);
    RCLCPP_INFO(this->get_logger(), "Bucket controller node initialized");
}

void BucketController::_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    actuator_msgs::msg::Actuators actuator_msg;
    actuator_msg.header.stamp = this->now();

    static constexpr uint8_t LEFT_STICK_X_AXIS = 0;
    static constexpr uint8_t LEFT_STICK_Y_AXIS = 1;
    static constexpr uint8_t RIGHT_STICK_X_AXIS = 2;
    static constexpr uint8_t RIGHT_STICK_Y_AXIS = 3;

    static constexpr uint8_t BUTTON_A = 0;

    if (msg->axes.size() < 4 || msg->buttons.size() < 1)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Joystick message has invalid size");
        return;
    }

    constexpr double ACTUATOR_SPEED = 0.1;  // m/s
    constexpr double ROTATE_SPEED = 1.5;    // rad/s

    actuator_msg.velocity.push_back(msg->axes[LEFT_STICK_Y_AXIS] * ACTUATOR_SPEED);
    actuator_msg.velocity.push_back(msg->axes[RIGHT_STICK_Y_AXIS] * ACTUATOR_SPEED);
    actuator_msg.velocity.push_back(msg->axes[RIGHT_STICK_X_AXIS] * ACTUATOR_SPEED);
    actuator_msg.velocity.push_back(msg->axes[LEFT_STICK_X_AXIS] * ROTATE_SPEED);

    // note: inverted, so magnet is released when the button's pressed
    actuator_msg.normalized.push_back(!msg->buttons[BUTTON_A]);

    // Publish actuator message
    _actuator_publisher->publish(actuator_msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<BucketController>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting bucket controller node");
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running bucket controller: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}

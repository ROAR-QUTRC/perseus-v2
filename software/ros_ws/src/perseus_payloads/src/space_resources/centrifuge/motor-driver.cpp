#include "space_resources/centrifuge/motor-driver.hpp"

CentrifugeDriver::CentrifugeDriver(const rclcpp::NodeOptions& options)
    : Node("centrifuge_driver", options)
{
    _centrifuge_speed_publisher = this->create_publisher<actuator_msgs::msg::ActuatorsAngularVelocity>("/centrifuge/speed", 10);
    RCLCPP_INFO(this->get_logger(), "Centrifuge driver node started");

    // example datat to test webui
    auto msg = actuator_msgs::msg::ActuatorsAngularVelocity([10.5]);
    // msg.header.stamp = this->now();
    // msg.header.frame_id = "base_link";

    // // Example: 4 wheels turning at different speeds
    // msg.angular_velocities.push_back(10.5);  // Front left
    // msg.angular_velocities.push_back(10.5);  // Front right
    // msg.angular_velocities.push_back(-5.0);  // Rear left (reverse)
    // msg.angular_velocities.push_back(15.0);  // Rear right

    _centrifuge_speed_publisher->publish(msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CentrifugeDriver>());
    rclcpp::shutdown();
    return 0;
}

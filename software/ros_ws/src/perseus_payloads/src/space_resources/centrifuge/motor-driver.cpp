#include "space_resources/centrifuge/motor-driver.hpp"

CentrifugeDriver::CentrifugeDriver(const rclcpp::NodeOptions& options)
    : Node("centrifuge_driver", options)
{
    _centrifuge_speed_publisher = this->create_publisher<actuator_msgs::msg::ActuatorsAngularVelocity>("/centrifuge/speed", 10);
    RCLCPP_INFO(this->get_logger(), "Centrifuge driver node started");

    // example data to test webui
    auto msg = actuator_msgs::msg::ActuatorsAngularVelocity();
    msg.velocity = {35.0};
    _centrifuge_speed_publisher->publish(msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CentrifugeDriver>());
    rclcpp::shutdown();
    return 0;
}

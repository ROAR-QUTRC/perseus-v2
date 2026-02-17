#include "space_resources/space_resources_controller/main.hpp"

#include <memory>
SpaceResourcesController::SpaceResourcesController(const rclcpp::NodeOptions& options)
    : rclcpp::Node("space_resources_controller", options)
{
    _water_concentration_service = this->create_client<perseus_interfaces::srv::Concentration>("/water/concentration");
    _ilmenite_concentration_service = this->create_client<perseus_interfaces::srv::Concentration>("/ilmenite/concentration");
    _centrifuge_control_publisher = this->create_publisher<actuator_msgs::msg::ActuatorsAngularVelocity>("/centrifuge/control", 10);
    _centrifuge_brake_publisher = this->create_publisher<std_msgs::msg::Bool>("/centrifuge/brake", 10);
    _centrifuge_speed_callback = this->create_subscription<actuator_msgs::msg::ActuatorsAngularVelocity>("/centrifuge/speed", 10, std::bind(&SpaceResourcesController::_speed_callback, this, std::placeholders::_1));
}

void SpaceResourcesController::_speed_callback(const actuator_msgs::msg::ActuatorsAngularVelocity msg)
{
    _current_velocity = msg.velocity.at(0);
    if (_current_velocity != _command_velocity)
    {
        actuator_msgs::msg::ActuatorsAngularVelocity velocity_message;
        velocity_message.set__velocity({_command_velocity});
        _centrifuge_control_publisher->publish(velocity_message);
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<SpaceResourcesController>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Space Resources Controller node");
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running Space Resources Controller node: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}

#include "arm_controller/arm_controller.hpp"

ArmController::ArmController(const rclcpp::NodeOptions& options)
    : Node("arm_controller", options)
{
    RCLCPP_INFO(this->get_logger(), "Arm Controller initialized");
}

void ArmController::cleanup()
{
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<ArmController>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Arm Controller node");
        rclcpp::spin(node);
        node->cleanup();
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Unhandled exception in Arm Controller: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
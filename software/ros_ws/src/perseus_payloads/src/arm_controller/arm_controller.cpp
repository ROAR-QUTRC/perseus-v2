#include "arm_controller/arm_controller.hpp"

#include "rclcpp/rclcpp.hpp"

ArmController::ArmController(const rclcpp::NodeOptions& options)
    : Node("arm_controller", options)
{
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<ArmController>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting arm driver node...");
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running arm driver: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}

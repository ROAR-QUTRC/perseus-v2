#include "arm_driver/arm_driver.hpp"

#include "rclcpp/rclcpp.hpp"

ArmDriver::ArmDriver(const rclcpp::NodeOptions& options)
    : Node("arm_driver", options)
{
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<ArmDriver>();
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

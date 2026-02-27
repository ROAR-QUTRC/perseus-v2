#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "ina228_driver/ina228_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<ina228_driver::Ina228Node>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("ina228_main"),
                     "Failed to start INA228 node: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}

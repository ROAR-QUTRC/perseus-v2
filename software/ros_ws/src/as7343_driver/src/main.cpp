#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "as7343_driver/as7343_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<as7343_driver::As7343Node>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("as7343_main"), "Failed to start AS7343 node: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}

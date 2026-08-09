/// @file dust_filter_node.cpp
/// @brief Entry point for the dust filter node.

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "perseus_sensors/dust_filter/dust_filter.hpp"

/// @brief Spins the dust filter until the node is shut down.
/// @param argc Argument count passed to ROS for command line parameter parsing.
/// @param argv Argument values passed to ROS for command line parameter parsing.
/// @return Zero on a clean shutdown.
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<perseus_sensors::DustFilter>());
    rclcpp::shutdown();
    return 0;
}

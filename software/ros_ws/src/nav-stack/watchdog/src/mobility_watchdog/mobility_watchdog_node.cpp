/// @file mobility_watchdog_node.cpp
/// @brief Entry point for the mobility watchdog node.

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "watchdog/mobility_watchdog/mobility_watchdog.hpp"

/// @brief Spins the mobility watchdog until the node is shut down.
/// @param argc Argument count passed to ROS for command line parameter parsing.
/// @param argv Argument values passed to ROS for command line parameter parsing.
/// @return Zero on a clean shutdown.
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<watchdog::MobilityWatchdog>());
    rclcpp::shutdown();
    return 0;
}

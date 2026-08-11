/// @file flat_footprint_broadcaster_node.cpp
/// @brief Entry point for the flat footprint broadcaster node.

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "footprint_broadcaster/flat_footprint_broadcaster/flat_footprint_broadcaster.hpp"

/// @brief Spins the flat footprint broadcaster until the node is shut down.
/// @param argc Argument count passed to ROS for command line parameter parsing.
/// @param argv Argument values passed to ROS for command line parameter parsing.
/// @return Zero on a clean shutdown.
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<footprint_broadcaster::FlatFootprintBroadcaster>());
    rclcpp::shutdown();
    return 0;
}

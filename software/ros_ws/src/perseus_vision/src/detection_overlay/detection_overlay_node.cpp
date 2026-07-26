/// @file detection_overlay_node.cpp
/// @brief Entry point for the detection overlay node.

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "perseus_vision/detection_overlay/detection_overlay.hpp"

/// @brief Spins the detection overlay until the node is shut down.
/// @param argc Argument count passed to ROS for command line parameter parsing.
/// @param argv Argument values passed to ROS for command line parameter parsing.
/// @return Zero on a clean shutdown.
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<perseus_vision::DetectionOverlay>());
    rclcpp::shutdown();
    return 0;
}

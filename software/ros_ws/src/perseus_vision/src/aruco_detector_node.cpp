/// @file aruco_detector_node.cpp
/// @brief Entry point for the ArUco marker detection node.

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "perseus_vision/aruco_detector.hpp"

/// @brief Spins the ArUco detector until the node is shut down.
/// @param argc Argument count passed to ROS for command line parameter parsing.
/// @param argv Argument values passed to ROS for command line parameter parsing.
/// @return Zero on a clean shutdown.
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<perseus_vision::ArucoDetector>());
    rclcpp::shutdown();
    return 0;
}

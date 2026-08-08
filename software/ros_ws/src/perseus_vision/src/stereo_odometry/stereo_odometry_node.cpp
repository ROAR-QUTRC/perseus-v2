/// @file stereo_odometry_node.cpp
/// @brief Entry point for the stereo odometry node.

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "perseus_vision/stereo_odometry/stereo_odometry.hpp"

/// @brief Spins the stereo odometry node until it is shut down.
/// @param argc Argument count passed to ROS for command line parameter parsing.
/// @param argv Argument values passed to ROS for command line parameter parsing.
/// @return Zero on a clean shutdown.
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<perseus_vision::StereoOdometry>());
    rclcpp::shutdown();
    return 0;
}

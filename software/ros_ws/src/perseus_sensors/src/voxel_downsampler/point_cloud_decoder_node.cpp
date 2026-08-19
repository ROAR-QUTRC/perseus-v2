/// @file point_cloud_decoder_node.cpp
/// @brief Entry point for the point cloud decoder node.

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "perseus_sensors/voxel_downsampler/point_cloud_decoder.hpp"

/// @brief Spins the point cloud decoder until the node is shut down.
/// @param argc Argument count passed to ROS for command line parameter parsing.
/// @param argv Argument values passed to ROS for command line parameter parsing.
/// @return Zero on a clean shutdown.
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<perseus_sensors::PointCloudDecoder>());
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp/rclcpp.hpp>

#include "lidar_segmentation/lidar_segmentation.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarSegmentation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#ifndef LIDAR_SEGMENTATION_HPP
#define LIDAR_SEGMENTATION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class LidarSegmentation : public rclcpp::Node
{
public:
    LidarSegmentation();

private:
    // Subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_ground_pub_;

    // Parameters
    std::string input_cloud_topic_;
    std::string ground_cloud_topic_;
    std::string non_ground_cloud_topic_;
    int32_t ransac_max_iterations_;
    double ransac_distance_threshold_;
    int model_type_;
    double axis_x_, axis_y_, axis_z_;
    double eps_angle_deg_;

    // Callback
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

#endif  // LIDAR_SEGMENTATION_HPP

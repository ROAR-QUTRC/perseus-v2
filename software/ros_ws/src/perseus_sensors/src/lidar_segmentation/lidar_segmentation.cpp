#include "lidar_segmentation/lidar_segmentation.hpp"

#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>  // for fromROSMsg/toROSMsg

LidarSegmentation::LidarSegmentation()
    : Node("lidar_segmentation_node")
{
    // Declare parameters with defaults
    this->declare_parameter<std::string>("input_cloud_topic", "/livox/lidar");
    this->declare_parameter<std::string>("ground_cloud_topic", "/livox/lidar/ground");
    this->declare_parameter<std::string>("non_ground_cloud_topic", "/livox/lidar/non_ground");
    this->declare_parameter<int32_t>("ransac_max_iterations", 100);
    this->declare_parameter<double>("ransac_distance_threshold", 0.2);
    this->declare_parameter<int>("model_type", 1);
    this->declare_parameter<double>("axis_x", 0.0);
    this->declare_parameter<double>("axis_y", 0.0);
    this->declare_parameter<double>("axis_z", 1.0);
    this->declare_parameter<double>("eps_angle_deg", 15.0);

    // Get parameters
    this->get_parameter("input_cloud_topic", input_cloud_topic_);
    this->get_parameter("ground_cloud_topic", ground_cloud_topic_);
    this->get_parameter("non_ground_cloud_topic", non_ground_cloud_topic_);
    this->get_parameter("ransac_max_iterations", ransac_max_iterations_);
    this->get_parameter("ransac_distance_threshold", ransac_distance_threshold_);
    this->get_parameter("model_type", model_type_);
    this->get_parameter("axis_x", axis_x_);
    this->get_parameter("axis_y", axis_y_);
    this->get_parameter("axis_z", axis_z_);
    this->get_parameter("eps_angle_deg", eps_angle_deg_);

    // Setup subscriber and publishers
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, 1, std::bind(&LidarSegmentation::cloud_callback, this, std::placeholders::_1));

    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(ground_cloud_topic_, 1);
    non_ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(non_ground_cloud_topic_, 1);

    RCLCPP_INFO(this->get_logger(), "LidarSegmentation node initialised with topics: %s, %s, %s",
                input_cloud_topic_.c_str(), ground_cloud_topic_.c_str(), non_ground_cloud_topic_.c_str());
}

void LidarSegmentation::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(Eigen::Vector3f(axis_x_, axis_y_, axis_z_));
    seg.setEpsAngle(eps_angle_deg_ * M_PI / 180.0);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(ransac_max_iterations_);
    seg.setDistanceThreshold(ransac_distance_threshold_);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty())
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No ground plane found.");
        return;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract_ground;
    extract_ground.setInputCloud(cloud);
    extract_ground.setIndices(inliers);
    extract_ground.setNegative(false);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract_ground.filter(*ground_cloud);

    pcl::ExtractIndices<pcl::PointXYZ> extract_non_ground;
    extract_non_ground.setInputCloud(cloud);
    extract_non_ground.setIndices(inliers);
    extract_non_ground.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract_non_ground.filter(*non_ground_cloud);

    sensor_msgs::msg::PointCloud2 ground_msg, non_ground_msg;
    pcl::toROSMsg(*ground_cloud, ground_msg);
    pcl::toROSMsg(*non_ground_cloud, non_ground_msg);

    ground_msg.header = msg->header;
    non_ground_msg.header = msg->header;

    ground_pub_->publish(ground_msg);
    non_ground_pub_->publish(non_ground_msg);
}

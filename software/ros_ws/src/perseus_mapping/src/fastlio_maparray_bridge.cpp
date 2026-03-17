#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <lidarslam_msgs/msg/map_array.hpp>
#include <lidarslam_msgs/msg/sub_map.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class FastLIOMapArrayBridge : public rclcpp::Node
{
public:
    FastLIOMapArrayBridge()
        : Node("fastlio_maparray_bridge"),
          total_distance_(0.0),
          has_last_pose_(false)
    {
        this->declare_parameter<double>("trans_for_mapupdate", 0.5);
        this->declare_parameter<int>("map_array_queue_size", 10);

        trans_for_mapupdate_ = this->get_parameter("trans_for_mapupdate").as_double();

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 10,
            std::bind(&FastLIOMapArrayBridge::cloud_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            std::bind(&FastLIOMapArrayBridge::odom_callback, this, std::placeholders::_1));

        map_array_pub_ = this->create_publisher<lidarslam_msgs::msg::MapArray>(
            "/map_array", 10);

        RCLCPP_INFO(this->get_logger(), "FastLIO MapArray Bridge started, update every %.2fm", trans_for_mapupdate_);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Odom callback fired");
        latest_pose_ = msg->pose.pose;
        latest_odom_stamp_ = msg->header.stamp;
        has_last_pose_ = true;
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Cloud callback fired, has_last_pose: %s", has_last_pose_ ? "true" : "false");

        if (!has_last_pose_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Waiting for odometry...");
            return;
        }

        // Check if robot has moved enough to add a new submap
        if (!map_array_.submaps.empty())
        {
            const auto& last_pose = map_array_.submaps.back().pose;
            double dx = latest_pose_.position.x - last_pose.position.x;
            double dy = latest_pose_.position.y - last_pose.position.y;
            double dz = latest_pose_.position.z - last_pose.position.z;
            double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (dist < trans_for_mapupdate_)
            {
                return;
            }
            total_distance_ += dist;
        }

        // Build new submap
        lidarslam_msgs::msg::SubMap submap;
        submap.header = msg->header;
        submap.pose = latest_pose_;
        submap.cloud = *msg;
        submap.distance = total_distance_;

        // Append to map array
        map_array_.header = msg->header;
        map_array_.cloud_coordinate = lidarslam_msgs::msg::MapArray::LOCAL;
        map_array_.submaps.push_back(submap);

        map_array_pub_->publish(map_array_);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "MapArray updated: %zu submaps, total distance: %.2fm",
                             map_array_.submaps.size(), total_distance_);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<lidarslam_msgs::msg::MapArray>::SharedPtr map_array_pub_;

    lidarslam_msgs::msg::MapArray map_array_;
    geometry_msgs::msg::Pose latest_pose_;
    rclcpp::Time latest_odom_stamp_;
    double total_distance_;
    double trans_for_mapupdate_;
    bool has_last_pose_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FastLIOMapArrayBridge>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
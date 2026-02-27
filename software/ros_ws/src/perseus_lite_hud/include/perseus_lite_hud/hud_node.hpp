#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <image_transport/image_transport.hpp>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "perseus_lite_hud/hud_elements/bearing_compass.hpp"
#include "perseus_lite_hud/hud_elements/lidar_proximity.hpp"
#include "perseus_lite_hud/hud_elements/odometer.hpp"
#include "perseus_lite_hud/hud_elements/pip_map.hpp"
#include "perseus_lite_hud/hud_elements/velocity_gauge.hpp"
#include "perseus_lite_hud/hud_renderer.hpp"

namespace perseus_lite_hud
{

    class HudOverlayNode : public rclcpp::Node
    {
    public:
        explicit HudOverlayNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        void _initialize_parameters();
        void _initialize_subscriptions();
        void _initialize_publisher();

        // Callbacks
        void _image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
        void _odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
        void _odom_filtered_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
        void _imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
        void _scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
        void _map_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg);
        void _cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr& msg);

        // Extract yaw from quaternion
        static double _yaw_from_quaternion(double x, double y, double z, double w);

        // Publisher
        image_transport::Publisher image_pub_;

        // Subscriptions
        image_transport::Subscriber image_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_filtered_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

        // TF
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // HUD
        HudRenderer renderer_;
        std::shared_ptr<BearingCompass> compass_;
        std::shared_ptr<PipMap> pip_map_;
        std::shared_ptr<LidarProximity> lidar_prox_;
        std::shared_ptr<VelocityGauge> velocity_gauge_;
        std::shared_ptr<Odometer> odometer_;

        // Cached data (protected by mutex)
        std::mutex data_mutex_;
        double heading_deg_ = 0.0;
        bool has_heading_ = false;
        double linear_vel_ = 0.0;
        double angular_vel_ = 0.0;
        bool has_velocity_ = false;
        std::array<double, 8> lidar_sectors_{};
        bool has_lidar_ = false;
        double odom_x_ = 0.0;
        double odom_y_ = 0.0;
        bool has_odom_ = false;

        // Parameters
        std::string map_frame_ = "map";
        std::string base_frame_ = "base_link";
        std::string heading_source_ = "odom";
    };

}  // namespace perseus_lite_hud

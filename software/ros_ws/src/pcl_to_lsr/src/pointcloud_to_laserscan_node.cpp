/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include "pcl_to_lsr/pointcloud_to_laserscan_node.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "Eigen/Eigenvalues"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace pointcloud_to_laserscan
{
    namespace
    {
        std::optional<Eigen::Vector3d> estimate_plane_normal(
            const sensor_msgs::msg::PointCloud2& cloud_msg,
            double min_height,
            double max_height)
        {
            std::vector<Eigen::Vector3d> plane_points;
            plane_points.reserve(cloud_msg.width * cloud_msg.height);

            for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x"),
                 iter_y(cloud_msg, "y"), iter_z(cloud_msg, "z");
                 iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
            {
                if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
                {
                    continue;
                }

                if (*iter_z > max_height || *iter_z < min_height)
                {
                    continue;
                }

                plane_points.emplace_back(*iter_x, *iter_y, *iter_z);
            }

            if (plane_points.size() < 3)
            {
                return std::nullopt;
            }

            Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
            for (const Eigen::Vector3d& point : plane_points)
            {
                centroid += point;
            }
            centroid /= static_cast<double>(plane_points.size());

            Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
            for (const Eigen::Vector3d& point : plane_points)
            {
                const Eigen::Vector3d centered_point = point - centroid;
                covariance += centered_point * centered_point.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
            if (solver.info() != Eigen::Success)
            {
                return std::nullopt;
            }

            Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
            if (normal.z() < 0.0)
            {
                normal = -normal;
            }

            return normal;
        }

        std::shared_ptr<sensor_msgs::msg::PointCloud2> rotate_cloud(
            const sensor_msgs::msg::PointCloud2& cloud_msg,
            const Eigen::Quaterniond& rotation)
        {
            auto rotated_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(cloud_msg);
            sensor_msgs::PointCloud2Iterator<float> out_x(*rotated_cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> out_y(*rotated_cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> out_z(*rotated_cloud, "z");

            for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x"),
                 iter_y(cloud_msg, "y"), iter_z(cloud_msg, "z");
                 iter_x != iter_x.end();
                 ++iter_x, ++iter_y, ++iter_z, ++out_x, ++out_y, ++out_z)
            {
                if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
                {
                    *out_x = *iter_x;
                    *out_y = *iter_y;
                    *out_z = *iter_z;
                    continue;
                }

                const Eigen::Vector3d rotated_point =
                    rotation * Eigen::Vector3d(*iter_x, *iter_y, *iter_z);
                *out_x = static_cast<float>(rotated_point.x());
                *out_y = static_cast<float>(rotated_point.y());
                *out_z = static_cast<float>(rotated_point.z());
            }

            return rotated_cloud;
        }
    }  // namespace

    PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node("pointcloud_to_laserscan", options)
    {
        target_frame_ = this->declare_parameter("target_frame", "livox_frame");
        tolerance_ = this->declare_parameter("transform_tolerance", 0.01);
        // TODO(hidmic): adjust default input queue size based on actual concurrency levels
        // achievable by the associated executor
        input_queue_size_ = this->declare_parameter(
            "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
        min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
        max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
        angle_min_ = this->declare_parameter("angle_min", -M_PI);
        angle_max_ = this->declare_parameter("angle_max", M_PI);
        angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
        scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
        range_min_ = this->declare_parameter("range_min", 0.0);
        range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
        inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
        use_inf_ = this->declare_parameter("use_inf", true);
        cloud_in_ = this->declare_parameter("cloud_in", "/livox/lidar");
        scan_out_ = this->declare_parameter("scan_out", "/livox/scan");
        imu_frame_ = this->declare_parameter("use_dynamic_conversion.imu_frame", "livox_frame");
        imu_topic_ = this->declare_parameter("use_dynamic_conversion.imu_topic", "/livox/imu/corrected");
        tilt_threshold_ = this->declare_parameter("tilt_threshold", 0.1745);                        // ~10 degrees
        use_dynamic_conversion_ = this->declare_parameter("use_dynamic_conversion.enabled", true);  // Enable tilt-based adjustment by default
        pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_out_, rclcpp::SensorDataQoS());

        // Subscribe to IMU topic
        rclcpp::SensorDataQoS qos;
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, qos,
            std::bind(&PointCloudToLaserScanNode::imu_callback, this, std::placeholders::_1));

        using std::placeholders::_1;
        // if pointcloud target frame specified, we need to filter by transform availability
        if (!target_frame_.empty())
        {
            tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
                this->get_node_base_interface(), this->get_node_timers_interface());
            tf2_->setCreateTimerInterface(timer_interface);
            tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
            message_filter_ = std::make_unique<MessageFilter>(
                sub_, *tf2_, target_frame_, input_queue_size_,
                this->get_node_logging_interface(),
                this->get_node_clock_interface());
            message_filter_->registerCallback(
                std::bind(&PointCloudToLaserScanNode::cloud_callback, this, _1));
        }
        else
        {  // otherwise setup direct subscription
            sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloud_callback, this, _1));
        }

        subscription_listener_thread_ = std::thread(
            std::bind(&PointCloudToLaserScanNode::subscription_listener_thread_loop, this));
    }

    PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
    {
        alive_.store(false);
        subscription_listener_thread_.join();
    }

    void PointCloudToLaserScanNode::imu_callback(sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
    {
        // Extract roll, pitch, yaw from the IMU quaternion
        double roll, pitch, yaw;
        quaternion_to_euler(
            imu_msg->orientation.x,
            imu_msg->orientation.y,
            imu_msg->orientation.z,
            imu_msg->orientation.w,
            roll, pitch, yaw);

        // Store the IMU orientation data with mutex protection
        {
            std::lock_guard<std::mutex> lock(imu_data_mutex_);
            imu_roll_ = roll;
            imu_pitch_ = pitch;
            imu_yaw_ = yaw;
        }

        RCLCPP_DEBUG(
            this->get_logger(),
            "IMU: roll=%.2f, pitch=%.2f, yaw=%.2f degrees",
            roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
    }

    void PointCloudToLaserScanNode::quaternion_to_euler(
        double qx, double qy, double qz, double qw,
        double& roll, double& pitch, double& yaw)
    {
        // Convert quaternion to Euler angles (roll, pitch, yaw)
        // Using the formula for ZYX convention (yaw, pitch, roll)

        // Roll (x-axis rotation)
        double sinr_cosp = 2.0 * (qw * qx + qy * qz);
        double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
        roll = atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        double sinp = 2.0 * (qw * qy - qz * qx);
        if (std::abs(sinp) >= 1.0)
            pitch = std::copysign(M_PI / 2.0, sinp);  // Use 90 degrees if out of range
        else
            pitch = asin(sinp);

        // Yaw (z-axis rotation)
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        yaw = atan2(siny_cosp, cosy_cosp);
    }

    void PointCloudToLaserScanNode::subscription_listener_thread_loop()
    {
        rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();

        const std::chrono::milliseconds timeout(100);
        while (rclcpp::ok(context) && alive_.load())
        {
            int subscription_count = pub_->get_subscription_count() +
                                     pub_->get_intra_process_subscription_count();
            if (subscription_count > 0)
            {
                if (!sub_.getSubscriber())
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Got a subscriber to laserscan, starting pointcloud subscriber");
                    rclcpp::SensorDataQoS qos;
                    qos.keep_last(input_queue_size_);
                    sub_.subscribe(this, cloud_in_, qos.get_rmw_qos_profile());
                }
            }
            else if (sub_.getSubscriber())
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "No subscribers to laserscan, shutting down pointcloud subscriber");
                sub_.unsubscribe();
            }
            rclcpp::Event::SharedPtr event = this->get_graph_event();
            this->wait_for_graph_change(event, timeout);
        }
        sub_.unsubscribe();
    }

    void PointCloudToLaserScanNode::cloud_callback(
        sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
    {
        // build laserscan output
        auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        scan_msg->header = cloud_msg->header;
        if (!target_frame_.empty())
        {
            scan_msg->header.frame_id = target_frame_;
        }

        scan_msg->angle_min = angle_min_;
        scan_msg->angle_max = angle_max_;
        scan_msg->angle_increment = angle_increment_;
        scan_msg->time_increment = 0.0;
        scan_msg->scan_time = scan_time_;
        scan_msg->range_min = range_min_;
        scan_msg->range_max = range_max_;

        // determine amount of rays to create
        uint32_t ranges_size = std::ceil(
            (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

        // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
        if (use_inf_)
        {
            scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
        }
        else
        {
            scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
        }

        // Transform cloud if necessary
        if (scan_msg->header.frame_id != cloud_msg->header.frame_id)
        {
            try
            {
                auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
                tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_));
                cloud_msg = cloud;
            }
            catch (tf2::TransformException& ex)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
                return;
            }
        }

        // Read current IMU orientation and compute adjusted height bounds for tilted case
        double current_pitch, current_roll;
        {
            std::lock_guard<std::mutex> lock(imu_data_mutex_);
            current_pitch = imu_pitch_;
            current_roll = imu_roll_;
        }

        // Calculate adjusted height thresholds based on IMU tilt
        // When the robot is tilted, the effective ground level shifts
        double adjusted_min_height = min_height_;
        double adjusted_max_height = max_height_;

        if (use_dynamic_conversion_)
        {
            double tilt_angle = std::sqrt(current_pitch * current_pitch + current_roll * current_roll);
            if (tilt_angle > tilt_threshold_)
            {
                // Robot is tilted - adjust height filtering
                // The ground appears higher/lower depending on pitch and roll
                double pitch_offset = std::sin(current_pitch) * 0.5;  // Scale factor for effect magnitude
                double roll_offset = std::sin(current_roll) * 0.5;
                double total_offset = std::sqrt(pitch_offset * pitch_offset + roll_offset * roll_offset);

                // Expand the height range when tilted to account for ground angle variation
                adjusted_min_height = min_height_ - total_offset;
                adjusted_max_height = max_height_ + total_offset;

                RCLCPP_DEBUG(
                    this->get_logger(),
                    "Robot tilted (%.2f°): adjusted height range [%.3f, %.3f]",
                    tilt_angle * 180.0 / M_PI, adjusted_min_height, adjusted_max_height);
            }
        }

        // Iterate through pointcloud
        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
             iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
             iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
            {
                RCLCPP_DEBUG(
                    this->get_logger(),
                    "rejected for nan in point(%f, %f, %f)\n",
                    *iter_x, *iter_y, *iter_z);
                continue;
            }

            if (*iter_z > adjusted_max_height || *iter_z < adjusted_min_height)
            {
                RCLCPP_DEBUG(
                    this->get_logger(),
                    "rejected for height %f not in range (%f, %f)\n",
                    *iter_z, adjusted_min_height, adjusted_max_height);
                continue;
            }

            double range = hypot(*iter_x, *iter_y);
            if (range < range_min_)
            {
                RCLCPP_DEBUG(
                    this->get_logger(),
                    "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
                    range, range_min_, *iter_x, *iter_y, *iter_z);
                continue;
            }
            if (range > range_max_)
            {
                RCLCPP_DEBUG(
                    this->get_logger(),
                    "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
                    range, range_max_, *iter_x, *iter_y, *iter_z);
                continue;
            }

            double angle = atan2(*iter_y, *iter_x);
            if (angle < scan_msg->angle_min || angle > scan_msg->angle_max)
            {
                RCLCPP_DEBUG(
                    this->get_logger(),
                    "rejected for angle %f not in range (%f, %f)\n",
                    angle, scan_msg->angle_min, scan_msg->angle_max);
                continue;
            }

            // overwrite range at laserscan ray if new range is smaller
            int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
            if (range < scan_msg->ranges[index])
            {
                scan_msg->ranges[index] = range;
            }
        }
        pub_->publish(std::move(scan_msg));
    }

}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)

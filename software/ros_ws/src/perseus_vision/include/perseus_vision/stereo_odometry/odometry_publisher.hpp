#pragma once

/// @file odometry_publisher.hpp
/// @brief Integrates incremental pose deltas and publishes odometry, pose, and TF.

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

namespace perseus_vision
{
    /// @brief Frame names, topic names, and behaviour flags for an OdometryPublisher.
    struct odometry_publisher_config_t
    {
        std::string odom_frame_id{"odom"};
        std::string base_link_frame_id{"base_link"};
        std::string sensor_frame_id{"camera"};
        std::string odometry_topic{"/perseus_vision/stereo_odometry/odometry"};
        std::string pose_topic{"/perseus_vision/stereo_odometry/pose"};
        // "~/" makes this private to the owning node, e.g. "/stereo_odometry/reset_pose"
        // rather than a bare "/reset_pose" shared across the whole graph.
        std::string reset_service_name{"~/reset_pose"};
        bool should_publish_tf{true};
        bool should_invert_tf{false};
        // When true, `set_initial_pose()`'s argument is the initial camera pose directly.
        // When false, it is the initial base_link pose, converted to the camera frame
        // via the base_link -> sensor_frame_id TF.
        bool is_initial_pose_in_camera_frame{false};
    };

    /// @brief Integrates a stream of incremental sensor-frame pose deltas into a
    ///        continuous base-frame pose, and publishes it as odometry, a pose, and TF.
    ///
    /// Composed into any node that estimates incremental motion (currently only
    /// StereoOdometry), keeping pose integration/publishing separate from how the
    /// motion delta itself is estimated. This can be reused as-is by a future
    /// incremental-pose sensor node.
    class OdometryPublisher
    {
    public:
        /// @brief Constructs the publisher and advertises its topics and reset service.
        /// @param node Node to create publishers, the TF broadcaster, and the service on.
        /// @param config Frame names, topic names, and behaviour flags.
        OdometryPublisher(rclcpp::Node& node, odometry_publisher_config_t config);

        /// @brief Sets the frame the sensor's motion deltas are measured in.
        /// @param frame_id Sensor frame ID, typically taken from an incoming message header.
        void set_sensor_frame_id(const std::string& frame_id);

        /// @brief Gets the frame the sensor's motion deltas are measured in.
        std::string get_sensor_frame_id() const;

        /// @brief Sets the pose covariance attached to the next published odometry message.
        void set_pose_covariance(const std::array<double, 36>& covariance);

        /// @brief Sets the twist covariance attached to the next published odometry message.
        void set_twist_covariance(const std::array<double, 36>& covariance);

        /// @brief Seeds the integrated pose, e.g. from a configured initial-pose parameter.
        /// @param pose Initial pose, interpreted per `is_initial_pose_in_camera_frame`.
        void set_initial_pose(const tf2::Transform& pose);

        /// @brief Integrates a new motion delta and publishes odometry, pose, and TF.
        /// @param delta_transform Incremental motion in the sensor frame since the last call.
        /// @param timestamp Timestamp of the sensor data the delta was estimated from.
        void integrate_and_publish(const tf2::Transform& delta_transform, const rclcpp::Time& timestamp);

    private:
        /// @brief Looks up the transform from base_link to the sensor frame.
        /// @param timestamp Time to look the transform up at, or time zero for the latest.
        /// @return The transform, or identity if it is not yet available.
        tf2::Transform _lookup_base_to_sensor(const rclcpp::Time& timestamp) const;

        /// @brief Resets the integrated pose to identity on request.
        void _handle_reset_request(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        rclcpp::Node& _node;
        odometry_publisher_config_t _config;

        tf2_ros::Buffer _tf_buffer;
        tf2_ros::TransformListener _tf_listener;
        std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odometry_publisher;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_publisher;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _reset_service;

        tf2::Transform _integrated_pose{tf2::Transform::getIdentity()};
        rclcpp::Time _last_update_time{0, 0, RCL_ROS_TIME};
        bool _has_last_update_time{false};

        std::array<double, 36> _pose_covariance{};
        std::array<double, 36> _twist_covariance{};
    };

}  // namespace perseus_vision

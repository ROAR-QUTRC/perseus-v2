#pragma once

/// @file flat_footprint_broadcaster.hpp
/// @brief Projects the 3D odom -> base_link transform onto the ground plane and
///        broadcasts it as odom -> base_footprint for nav2's planar costmaps.

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace footprint_broadcaster
{
    /// @brief ROS 2 node that flattens base_link's pose into a planar base_footprint frame.
    ///
    /// The EKF (see autonomy_bringup/config/ekf_config.yaml) runs with two_d_mode: false and
    /// publishes odom -> base_link carrying FAST-LIO's real z, roll and pitch. nav2's
    /// costmaps and controller assume a robot that moves on a plane, so this node looks up
    /// odom -> base_link on a timer, zeroes z/roll/pitch while keeping yaw, and broadcasts the
    /// result as odom -> base_footprint -- leaving base_link's full 3D pose untouched for
    /// everything else that needs it (e.g. the 3D localisation stack itself).
    class FlatFootprintBroadcaster : public rclcpp::Node
    {
    public:
        /// @brief Constructs the node, declaring parameters and setting up the TF listener,
        ///        broadcaster and publish timer.
        /// @param options Node options, supplied by main().
        explicit FlatFootprintBroadcaster(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        /// @brief Default parent frame that both base_link and base_footprint are published in.
        static inline const std::string DEFAULT_ODOM_FRAME = "odom";
        /// @brief Default frame carrying the full 3D pose.
        static inline const std::string DEFAULT_BASE_LINK_FRAME = "base_link";
        /// @brief Default frame this node publishes the flattened pose as.
        static inline const std::string DEFAULT_BASE_FOOTPRINT_FRAME = "base_footprint";
        /// @brief Default rate the flattened transform is (re)published at, in Hz.
        static constexpr double DEFAULT_PUBLISH_RATE_HZ = 30.0;

        /// @brief Declares every parameter and copies the frame names into their members.
        void _load_parameters();

        /// @brief Looks up odom -> base_link, flattens it, and broadcasts odom -> base_footprint.
        void _timer_callback();

        std::string _odom_frame{DEFAULT_ODOM_FRAME};
        std::string _base_link_frame{DEFAULT_BASE_LINK_FRAME};
        std::string _base_footprint_frame{DEFAULT_BASE_FOOTPRINT_FRAME};

        std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
        std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
        rclcpp::TimerBase::SharedPtr _timer;
    };

}  // namespace footprint_broadcaster

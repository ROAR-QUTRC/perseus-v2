/// @file flat_footprint_broadcaster.cpp
/// @brief Implementation of the base_footprint flattening broadcaster.

#include "footprint_broadcaster/flat_footprint_broadcaster/flat_footprint_broadcaster.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <functional>

namespace footprint_broadcaster
{
    FlatFootprintBroadcaster::FlatFootprintBroadcaster(const rclcpp::NodeOptions& options)
        : rclcpp::Node("flat_footprint_broadcaster", options)
    {
        _load_parameters();

        _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
        _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        const double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
        const auto period =
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_hz));
        _timer = this->create_wall_timer(
            period, std::bind(&FlatFootprintBroadcaster::_timer_callback, this));
    }

    void FlatFootprintBroadcaster::_load_parameters()
    {
        _odom_frame = this->declare_parameter("odom_frame", DEFAULT_ODOM_FRAME);
        _base_link_frame = this->declare_parameter("base_link_frame", DEFAULT_BASE_LINK_FRAME);
        _base_footprint_frame =
            this->declare_parameter("base_footprint_frame", DEFAULT_BASE_FOOTPRINT_FRAME);
        this->declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ);
    }

    void FlatFootprintBroadcaster::_timer_callback()
    {
        geometry_msgs::msg::TransformStamped odom_to_base_link;
        try
        {
            odom_to_base_link =
                _tf_buffer->lookupTransform(_odom_frame, _base_link_frame, tf2::TimePointZero);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Waiting for %s -> %s: %s",
                _odom_frame.c_str(),
                _base_link_frame.c_str(),
                ex.what());
            return;
        }

        // Only yaw survives: nav2's planar costmaps and controllers assume the robot moves on
        // the ground plane, so z, roll and pitch are dropped rather than passed through.
        const double yaw = tf2::getYaw(odom_to_base_link.transform.rotation);
        tf2::Quaternion flattened_rotation;
        flattened_rotation.setRPY(0.0, 0.0, yaw);

        geometry_msgs::msg::TransformStamped odom_to_base_footprint;
        odom_to_base_footprint.header.stamp = odom_to_base_link.header.stamp;
        odom_to_base_footprint.header.frame_id = _odom_frame;
        odom_to_base_footprint.child_frame_id = _base_footprint_frame;
        odom_to_base_footprint.transform.translation.x = odom_to_base_link.transform.translation.x;
        odom_to_base_footprint.transform.translation.y = odom_to_base_link.transform.translation.y;
        odom_to_base_footprint.transform.translation.z = 0.0;
        odom_to_base_footprint.transform.rotation = tf2::toMsg(flattened_rotation);

        _tf_broadcaster->sendTransform(odom_to_base_footprint);
    }

}  // namespace footprint_broadcaster

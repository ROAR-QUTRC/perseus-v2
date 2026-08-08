/// @file odometry_publisher.cpp
/// @brief Implementation of incremental pose integration and publishing.

#include "perseus_vision/stereo_odometry/odometry_publisher.hpp"

#include <tf2/exceptions.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>

namespace perseus_vision
{
    namespace
    {
        /// @brief Queue depth used for the odometry and pose topics.
        constexpr int QOS_DEPTH = 1;
        /// @brief How long to wait for the base_link -> sensor TF before assuming identity.
        constexpr double TF_LOOKUP_TIMEOUT_S = 0.1;
        /// @brief Minimum gap between repeated "TF not available" warnings, in milliseconds.
        constexpr int64_t TF_WARN_THROTTLE_MS = 10000;
    }  // namespace

    OdometryPublisher::OdometryPublisher(rclcpp::Node& node, odometry_publisher_config_t config)
        : _node(node),
          _config(std::move(config)),
          _tf_buffer(node.get_clock()),
          _tf_listener(_tf_buffer)
    {
        _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

        _odometry_publisher = node.create_publisher<nav_msgs::msg::Odometry>(_config.odometry_topic, QOS_DEPTH);
        _pose_publisher = node.create_publisher<geometry_msgs::msg::PoseStamped>(_config.pose_topic, QOS_DEPTH);

        _reset_service = node.create_service<std_srvs::srv::Trigger>(
            _config.reset_service_name,
            std::bind(&OdometryPublisher::_handle_reset_request, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(node.get_logger(),
                    "OdometryPublisher ready — odom_frame_id=%s base_link_frame_id=%s "
                    "sensor_frame_id=%s publish_tf=%s",
                    _config.odom_frame_id.c_str(), _config.base_link_frame_id.c_str(),
                    _config.sensor_frame_id.c_str(), _config.should_publish_tf ? "true" : "false");
    }

    void OdometryPublisher::set_sensor_frame_id(const std::string& frame_id)
    {
        _config.sensor_frame_id = frame_id;
    }

    std::string OdometryPublisher::get_sensor_frame_id() const
    {
        return _config.sensor_frame_id;
    }

    void OdometryPublisher::set_pose_covariance(const std::array<double, 36>& covariance)
    {
        _pose_covariance = covariance;
    }

    void OdometryPublisher::set_twist_covariance(const std::array<double, 36>& covariance)
    {
        _twist_covariance = covariance;
    }

    void OdometryPublisher::set_initial_pose(const tf2::Transform& pose)
    {
        if (_config.is_initial_pose_in_camera_frame)
        {
            _integrated_pose = pose;
            return;
        }

        // Time zero requests the latest available transform, since there is no sensor
        // timestamp yet to look this up at.
        const tf2::Transform base_to_sensor = _lookup_base_to_sensor(rclcpp::Time(0, 0, RCL_ROS_TIME));
        _integrated_pose = pose * base_to_sensor;
    }

    void OdometryPublisher::integrate_and_publish(const tf2::Transform& delta_transform, const rclcpp::Time& timestamp)
    {
        if (_config.sensor_frame_id.empty())
        {
            RCLCPP_ERROR(_node.get_logger(), "integrate_and_publish() called with unknown sensor frame id");
            return;
        }

        if (_has_last_update_time && timestamp < _last_update_time)
        {
            RCLCPP_WARN(_node.get_logger(),
                        "Saw negative time change in incoming sensor data, resetting integrated pose.");
            _integrated_pose = tf2::Transform::getIdentity();
        }

        // {odom}^T_{c_camera} = {odom}^T_{p_camera} * {p_camera}^T_{c_camera}
        _integrated_pose *= delta_transform;

        const tf2::Transform base_to_sensor = _lookup_base_to_sensor(timestamp);
        const tf2::Transform base_transform = _config.is_initial_pose_in_camera_frame
                                                  ? base_to_sensor * _integrated_pose * base_to_sensor.inverse()
                                                  : _integrated_pose * base_to_sensor.inverse();

        nav_msgs::msg::Odometry odometry_message;
        odometry_message.header.stamp = timestamp;
        odometry_message.header.frame_id = _config.odom_frame_id;
        odometry_message.child_frame_id = _config.base_link_frame_id;
        tf2::toMsg(base_transform, odometry_message.pose.pose);

        // Twist cannot be computed on the first update, as no delta_t is available yet.
        const tf2::Transform delta_base_transform = base_to_sensor * delta_transform * base_to_sensor.inverse();
        if (_has_last_update_time)
        {
            const double delta_t_s = (timestamp - _last_update_time).seconds();
            if (delta_t_s > 0.0)
            {
                odometry_message.twist.twist.linear.x = delta_base_transform.getOrigin().getX() / delta_t_s;
                odometry_message.twist.twist.linear.y = delta_base_transform.getOrigin().getY() / delta_t_s;
                odometry_message.twist.twist.linear.z = delta_base_transform.getOrigin().getZ() / delta_t_s;

                const tf2::Quaternion delta_rotation = delta_base_transform.getRotation();
                const tf2::Vector3 angular_twist = delta_rotation.getAxis() * delta_rotation.getAngle() / delta_t_s;
                odometry_message.twist.twist.angular.x = angular_twist.x();
                odometry_message.twist.twist.angular.y = angular_twist.y();
                odometry_message.twist.twist.angular.z = angular_twist.z();
            }
        }

        odometry_message.pose.covariance = _pose_covariance;
        odometry_message.twist.covariance = _twist_covariance;
        _odometry_publisher->publish(odometry_message);

        geometry_msgs::msg::PoseStamped pose_message;
        pose_message.header = odometry_message.header;
        pose_message.pose = odometry_message.pose.pose;
        _pose_publisher->publish(pose_message);

        if (_config.should_publish_tf)
        {
            geometry_msgs::msg::TransformStamped transform_message;
            transform_message.header.stamp = timestamp;

            if (_config.should_invert_tf)
            {
                transform_message.header.frame_id = _config.base_link_frame_id;
                transform_message.child_frame_id = _config.odom_frame_id;
                transform_message.transform = tf2::toMsg(base_transform.inverse());
            }
            else
            {
                transform_message.header.frame_id = _config.odom_frame_id;
                transform_message.child_frame_id = _config.base_link_frame_id;
                transform_message.transform = tf2::toMsg(base_transform);
            }
            _tf_broadcaster->sendTransform(transform_message);
        }

        _last_update_time = timestamp;
        _has_last_update_time = true;
    }

    tf2::Transform OdometryPublisher::_lookup_base_to_sensor(const rclcpp::Time& timestamp) const
    {
        tf2::Transform base_to_sensor = tf2::Transform::getIdentity();
        try
        {
            const geometry_msgs::msg::TransformStamped transform = _tf_buffer.lookupTransform(
                _config.base_link_frame_id, _config.sensor_frame_id, timestamp,
                tf2::durationFromSec(TF_LOOKUP_TIMEOUT_S));
            tf2::fromMsg(transform.transform, base_to_sensor);
        }
        catch (const tf2::TransformException& error)
        {
            RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), TF_WARN_THROTTLE_MS,
                                 "TF from '%s' to '%s' is not available, assuming identity: %s",
                                 _config.base_link_frame_id.c_str(), _config.sensor_frame_id.c_str(), error.what());
        }
        return base_to_sensor;
    }

    void OdometryPublisher::_handle_reset_request(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        _integrated_pose = tf2::Transform::getIdentity();
        _has_last_update_time = false;
        response->success = true;
        response->message = "Integrated pose reset to identity.";
    }

}  // namespace perseus_vision

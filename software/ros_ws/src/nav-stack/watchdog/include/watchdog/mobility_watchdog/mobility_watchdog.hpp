#pragma once

/// @file mobility_watchdog.hpp
/// @brief Detects sustained wheel slip by comparing commanded velocity against
///        LiDAR-inertial-filtered odometry.

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "perseus_interfaces/msg/mobility_status.hpp"
#include "watchdog/mobility_watchdog/slip_channel_monitor.hpp"

namespace watchdog
{
    /// @brief ROS 2 node that flags sustained wheel slip on loose terrain.
    ///
    /// Compares the velocity commanded on `/cmd_vel_out` against the velocity actually
    /// measured on `/odometry/filtered`. The filtered odometry is derived from FAST-LIO's
    /// LiDAR-inertial pose and the IMU gyro (see autonomy_bringup/config/ekf_config.yaml) --
    /// it contains no wheel-encoder input, so it keeps reporting the robot's true motion even
    /// while the wheels themselves are spinning freely in sand. A persistent gap between
    /// command and measurement on either channel is therefore genuine slip, not just two
    /// wheel-derived signals agreeing with each other for the wrong reason.
    class MobilityWatchdog : public rclcpp::Node
    {
    public:
        /// @brief Constructs the node, declaring parameters and setting up subscriptions
        ///        and the status publisher.
        /// @param options Node options, supplied by main().
        explicit MobilityWatchdog(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        /// @brief Default topic that commanded velocity is read from.
        static inline const std::string DEFAULT_CMD_VEL_TOPIC = "/cmd_vel_out";
        /// @brief Default topic that measured odometry is read from.
        static inline const std::string DEFAULT_ODOMETRY_TOPIC = "/odometry/filtered";
        /// @brief Default topic that the mobility status is published on.
        static inline const std::string DEFAULT_OUTPUT_TOPIC = "/watchdog/mobility_status";

        /// @brief Default minimum commanded forward speed evaluated for slip, in m/s.
        static constexpr double DEFAULT_MIN_LINEAR_SPEED_MPS = 0.15;
        /// @brief Default minimum commanded yaw rate evaluated for slip, in rad/s.
        static constexpr double DEFAULT_MIN_ANGULAR_SPEED_RPS = 0.2;
        /// @brief Default mean slip ratio, over the detection window, that counts as slipping.
        static constexpr double DEFAULT_SLIP_RATIO_THRESHOLD = 0.4;
        /// @brief Default length of the sliding window slip ratios are averaged over, in seconds.
        static constexpr double DEFAULT_DETECTION_WINDOW_S = 0.75;
        /// @brief Default grace period after a command starts before it is evaluated, in seconds.
        static constexpr double DEFAULT_ACTIVATION_GRACE_PERIOD_S = 0.4;
        /// @brief Default age beyond which a cached command is treated as stale, in seconds.
        static constexpr double DEFAULT_COMMAND_TIMEOUT_S = 0.5;

        /// @brief Declares every parameter, copying non-topic values into the matching
        ///        member and rebuilding the slip monitors from them.
        /// @param cmd_vel_topic_out Receives the resolved commanded velocity topic.
        /// @param odometry_topic_out Receives the resolved odometry topic.
        /// @param output_topic_out Receives the resolved status output topic.
        void _load_parameters(
            std::string& cmd_vel_topic_out,
            std::string& odometry_topic_out,
            std::string& output_topic_out);

        /// @brief Caches the most recently commanded velocity and when it arrived.
        /// @param msg Incoming commanded velocity message.
        void _cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

        /// @brief Compares the cached command against measured odometry and publishes the
        ///        result.
        /// @param msg Incoming filtered odometry message.
        void _odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        /// @brief Whether the cached command is recent enough to evaluate against.
        /// @param now Time to measure the command's age against.
        bool _is_cached_command_fresh(const rclcpp::Time& now) const;

        SlipChannelMonitor _linear_monitor{SlipChannelMonitor::config_t{
            .min_commanded_magnitude = DEFAULT_MIN_LINEAR_SPEED_MPS,
            .slip_ratio_threshold = DEFAULT_SLIP_RATIO_THRESHOLD,
            .detection_window_s = DEFAULT_DETECTION_WINDOW_S,
            .activation_grace_period_s = DEFAULT_ACTIVATION_GRACE_PERIOD_S,
        }};
        SlipChannelMonitor _angular_monitor{SlipChannelMonitor::config_t{
            .min_commanded_magnitude = DEFAULT_MIN_ANGULAR_SPEED_RPS,
            .slip_ratio_threshold = DEFAULT_SLIP_RATIO_THRESHOLD,
            .detection_window_s = DEFAULT_DETECTION_WINDOW_S,
            .activation_grace_period_s = DEFAULT_ACTIVATION_GRACE_PERIOD_S,
        }};

        double _command_timeout_s{DEFAULT_COMMAND_TIMEOUT_S};
        geometry_msgs::msg::Twist _latest_cmd_vel;
        rclcpp::Time _latest_cmd_vel_stamp;
        bool _has_cmd_vel{false};

        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr _cmd_vel_subscription;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_subscription;
        rclcpp::Publisher<perseus_interfaces::msg::MobilityStatus>::SharedPtr _status_publisher;
    };

}  // namespace watchdog

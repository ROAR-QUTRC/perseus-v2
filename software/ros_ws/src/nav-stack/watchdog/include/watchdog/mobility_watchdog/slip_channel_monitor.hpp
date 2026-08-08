#pragma once

/// @file slip_channel_monitor.hpp
/// @brief Sliding-window comparison of a commanded motion channel against what was measured.

#include <deque>
#include <rclcpp/time.hpp>

namespace watchdog
{
    /// @brief Tracks whether one motion channel (e.g. forward speed, or yaw rate) has been
    ///        sustainedly failing to track its commanded value, as happens when a wheel loses
    ///        traction in loose terrain.
    ///
    /// Fed one commanded/measured sample per update(), independent of any particular message
    /// type, so the same logic serves both the linear and angular channels of
    /// MobilityWatchdog without duplication.
    class SlipChannelMonitor
    {
    public:
        /// @brief Tunable behaviour of a single channel's slip assessment.
        struct config_t
        {
            /// @brief Commanded magnitude below which a sample is ignored entirely.
            ///
            /// Below this, the commanded motion is too small for slip to be distinguishable
            /// from ordinary sensor and control noise.
            double min_commanded_magnitude{0.0};
            /// @brief Mean slip ratio over the detection window at or above which the channel
            ///        is reported as slipping.
            double slip_ratio_threshold{0.0};
            /// @brief Duration of the sliding window slip ratios are averaged over, in seconds.
            double detection_window_s{0.0};
            /// @brief Grace period after commanding starts before evaluation begins, in seconds.
            ///
            /// Suppresses the transient mismatch every command incurs while the robot is still
            /// accelerating to match it, which would otherwise look identical to true slip.
            double activation_grace_period_s{0.0};
        };

        /// @brief Constructs a monitor with fixed thresholds and window sizing.
        /// @param config Behaviour to apply to every sample.
        explicit SlipChannelMonitor(config_t config);

        /// @brief Feeds one commanded/measured sample pair and updates the rolling assessment.
        /// @param commanded Signed commanded value for this channel, e.g. m/s or rad/s.
        /// @param measured Signed measured value, in the same units and reference frame.
        /// @param now Time this sample was taken, used to prune the window and time the grace
        ///        period.
        void update(double commanded, double measured, const rclcpp::Time& now);

        /// @brief Whether the channel currently has enough commanded, non-stale history to
        ///        report a meaningful slip assessment.
        /// @return True once the command has been active past the grace period.
        bool is_evaluating() const;

        /// @brief Whether the mean slip ratio over the window is at or above threshold.
        ///
        /// Only meaningful while is_evaluating() is true; false otherwise.
        bool is_slipping() const;

        /// @brief Mean slip ratio over the current window: 0 means commanded speed fully
        ///        achieved, 1 means no measured motion at all despite the command. Not clamped,
        ///        so it can go negative when the robot exceeds the commanded speed.
        /// @return The mean ratio, or 0.0 while not evaluating.
        double slip_ratio() const;

        /// @brief Signed commanded value from the most recent update().
        double commanded_value() const;

        /// @brief Signed measured value from the most recent update().
        double measured_value() const;

    private:
        /// @brief One windowed slip ratio measurement.
        struct sample_t
        {
            rclcpp::Time stamp;
            double ratio{0.0};
        };

        /// @brief Drops samples older than the detection window relative to @p now.
        /// @param now Current time used as the window's leading edge.
        void _prune_old_samples(const rclcpp::Time& now);

        config_t _config;
        std::deque<sample_t> _samples;
        bool _is_command_active{false};
        rclcpp::Time _command_active_since;
        bool _is_evaluating{false};
        bool _is_slipping{false};
        double _slip_ratio{0.0};
        double _last_commanded{0.0};
        double _last_measured{0.0};
    };

}  // namespace watchdog

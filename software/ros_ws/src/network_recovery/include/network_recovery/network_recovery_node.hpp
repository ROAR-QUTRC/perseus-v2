#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <perseus_interfaces/msg/network_recovery_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "network_recovery/heatmap.hpp"
#include "network_recovery/network_monitor.hpp"
#include "network_recovery/recovery_navigator.hpp"

namespace network_recovery
{

    // State constants matching the message definition.
    enum State : uint8_t
    {
        MONITORING = 0,
        SIGNAL_DEGRADED = 1,
        SIGNAL_LEASH = 2,
        SIGNAL_LOST_WAITING = 3,
        RECOVERING = 4,
        RECOVERY_SUCCEEDED = 5,
        RECOVERY_FAILED = 6,
    };

    class NetworkRecoveryNode : public rclcpp::Node
    {
    public:
        NetworkRecoveryNode();
        ~NetworkRecoveryNode() override;

    private:
        // Parameter handling
        struct Params
        {
            // Network monitoring
            std::string ping_target;
            double ping_interval_s{2.0};
            double ping_timeout_s{1.5};
            std::string wifi_interface;

            // Signal thresholds
            int rssi_warning_threshold{-70};
            int rssi_critical_threshold{-85};

            // Trend
            int trend_window_size{10};
            double trend_threshold{1.0};

            // Leash
            bool leash_enabled{true};
            int leash_rssi_threshold{-70};
            int leash_min_samples{5};
            bool leash_pause_robot{false};
            double leash_timeout_s{60.0};

            // Recovery
            double signal_loss_timeout_s{30.0};
            bool recovery_enabled{true};

            // Breadcrumbs
            int breadcrumb_max_count{500};
            double breadcrumb_min_distance_m{0.5};
            double breadcrumb_interval_s{5.0};
            int breadcrumb_pin_rssi_threshold{-55};
            int pinned_breadcrumb_max_count{50};
            int breadcrumb_strong_signal_threshold{-55};
            double breadcrumb_strong_distance_m{2.0};
            int breadcrumb_weak_signal_threshold{-70};
            double breadcrumb_weak_distance_m{0.3};

            // Heatmap
            double heatmap_cell_size_m{0.5};
            double heatmap_publish_rate_hz{0.5};
            double heatmap_marker_height{0.1};
            double heatmap_marker_z{0.05};
            double heatmap_marker_alpha{0.6};
            int heatmap_good_signal_threshold{-55};

            // Heatmap export
            bool heatmap_save_on_shutdown{true};
            std::string heatmap_save_directory{"~/.ros/network_recovery"};

            // Recovery nav
            int recovery_batch_size{3};
            double retreat_distance_m{3.0};
            std::string nav2_action_name{"/navigate_through_poses"};

            // Rates
            double pose_sample_rate_hz{2.0};
            double status_publish_rate_hz{1.0};
            double state_machine_rate_hz{2.0};

            // Frames
            std::string map_frame{"map"};
            std::string base_frame{"base_link"};
        };

        void declare_params();
        Params load_params();

        // TF
        std::optional<geometry_msgs::msg::PoseStamped> get_current_pose();

        // Timer callbacks
        void pose_sample_cb();
        void breadcrumb_cb();
        void status_publish_cb();
        void heatmap_publish_cb();
        void state_machine_tick();

        // State machine
        void tick_monitoring(bool connected, int rssi, SignalTrend trend, double now);
        void tick_degraded(bool connected, int rssi, SignalTrend trend, double now);
        void tick_leash(bool connected, int rssi, SignalTrend trend, double now);
        void tick_lost_waiting(bool connected, double now);
        void tick_recovering(bool connected);
        void tick_succeeded(double now);
        void tick_failed(bool connected);

        void begin_recovery();
        std::string build_status_message();
        void publish_trail();

        // Members
        Params p_;

        rclcpp::Publisher<perseus_interfaces::msg::NetworkRecoveryStatus>::SharedPtr
            status_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
            heatmap_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trail_pub_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::unique_ptr<NetworkMonitor> monitor_;
        std::unique_ptr<HeatmapGrid> heatmap_;
        std::shared_ptr<RecoveryNavigator> navigator_;

        // State
        State state_{MONITORING};
        double signal_lost_time_{0.0};
        double leash_enter_time_{0.0};
        int leash_degrading_count_{0};
        double recovery_succeeded_time_{0.0};
        std::string recovery_status_msg_;

        // Breadcrumbs
        std::deque<Breadcrumb> breadcrumbs_;
        std::vector<Breadcrumb> pinned_breadcrumbs_;
        std::optional<geometry_msgs::msg::PoseStamped> last_breadcrumb_pose_;
        bool trail_dirty_{false};

        // Timers
        rclcpp::TimerBase::SharedPtr pose_timer_;
        rclcpp::TimerBase::SharedPtr breadcrumb_timer_;
        rclcpp::TimerBase::SharedPtr status_timer_;
        rclcpp::TimerBase::SharedPtr heatmap_timer_;
        rclcpp::TimerBase::SharedPtr state_timer_;
    };

}  // namespace network_recovery

#pragma once

#include <cstdint>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vector>

namespace network_recovery
{

    enum class RecoveryPhase : uint8_t
    {
        IDLE = 0,
        GRADUAL_RETREAT = 1,
        HEATMAP_GUIDED = 2,
        FULL_BACKTRACK = 3,
        DONE = 4,
    };

    /// A breadcrumb: pose + RSSI at that point.
    struct Breadcrumb
    {
        geometry_msgs::msg::PoseStamped pose;
        int rssi_dbm{0};
    };

    /// Manages Nav2 NavigateThroughPoses for three-phase recovery.
    class RecoveryNavigator
    {
    public:
        using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
        using GoalHandle =
            rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

        struct Config
        {
            std::string action_name{"/navigate_through_poses"};
            int batch_size{3};
            double retreat_distance_m{3.0};
            std::string map_frame{"map"};
        };

        RecoveryNavigator(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
                          rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
                          rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
                          rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables,
                          rclcpp::Clock::SharedPtr clock,
                          const Config& cfg);

        RecoveryPhase phase() const { return phase_; }
        bool is_active() const;
        bool is_goal_in_progress() const;

        void cancel_all_goals();
        void reset();

        /// Begin three-phase recovery. Returns initial status message.
        std::string start_recovery(
            const std::vector<Breadcrumb>& breadcrumbs,
            const std::optional<geometry_msgs::msg::PoseStamped>& heatmap_target);

        /// Check if current goal is done and advance phases.
        /// Returns {all_done, status_message}.
        std::pair<bool, std::string> check_phase_complete(
            const std::vector<Breadcrumb>& breadcrumbs,
            const std::optional<geometry_msgs::msg::PoseStamped>& heatmap_target);

    private:
        std::string advance_to_phase2(
            const std::vector<Breadcrumb>& breadcrumbs,
            const std::optional<geometry_msgs::msg::PoseStamped>& heatmap_target);
        std::string advance_to_phase3(const std::vector<Breadcrumb>& breadcrumbs);

        std::vector<geometry_msgs::msg::PoseStamped> get_retreat_poses(
            const std::vector<Breadcrumb>& breadcrumbs) const;

        void send_goal(const std::vector<geometry_msgs::msg::PoseStamped>& poses);

        std::string current_status_message() const;

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
        rclcpp::Logger logger_;
        rclcpp::Clock::SharedPtr clock_;
        Config cfg_;

        rclcpp_action::Client<NavigateThroughPoses>::SharedPtr action_client_;

        RecoveryPhase phase_{RecoveryPhase::IDLE};

        mutable std::mutex goal_mtx_;
        std::shared_ptr<GoalHandle> goal_handle_;
        bool goal_done_{false};
        bool goal_succeeded_{false};

        // Backtrack state
        std::vector<std::vector<geometry_msgs::msg::PoseStamped>> backtrack_batches_;
        size_t current_batch_idx_{0};
    };

}  // namespace network_recovery

#include "network_recovery/recovery_navigator.hpp"

#include <cmath>

using namespace std::placeholders;

namespace network_recovery {

RecoveryNavigator::RecoveryNavigator(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables,
    rclcpp::Clock::SharedPtr clock,
    const Config& cfg)
    : node_base_(std::move(node_base)),
      logger_(node_logging->get_logger()),
      clock_(std::move(clock)),
      cfg_(cfg) {
  action_client_ =
      rclcpp_action::create_client<NavigateThroughPoses>(
          node_base_, node_graph, node_logging, node_waitables, cfg_.action_name);
}

bool RecoveryNavigator::is_active() const {
  return phase_ != RecoveryPhase::IDLE && phase_ != RecoveryPhase::DONE;
}

bool RecoveryNavigator::is_goal_in_progress() const {
  std::lock_guard<std::mutex> lk(goal_mtx_);
  return goal_handle_ != nullptr && !goal_done_;
}

void RecoveryNavigator::cancel_all_goals() {
  std::lock_guard<std::mutex> lk(goal_mtx_);
  if (goal_handle_ && !goal_done_) {
    RCLCPP_INFO(logger_, "Cancelling active Nav2 goal");
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_ = nullptr;
    goal_done_ = true;
  }
}

void RecoveryNavigator::reset() {
  cancel_all_goals();
  phase_ = RecoveryPhase::IDLE;
  backtrack_batches_.clear();
  current_batch_idx_ = 0;
}

std::string RecoveryNavigator::start_recovery(
    const std::vector<Breadcrumb>& breadcrumbs,
    const std::optional<geometry_msgs::msg::PoseStamped>& heatmap_target) {
  reset();

  if (breadcrumbs.empty()) {
    if (heatmap_target) {
      phase_ = RecoveryPhase::HEATMAP_GUIDED;
      send_goal({*heatmap_target});
      return "no breadcrumbs, navigating to nearest good-signal area";
    }
    phase_ = RecoveryPhase::DONE;
    return "no recovery data available";
  }

  // Phase 1: Gradual retreat
  phase_ = RecoveryPhase::GRADUAL_RETREAT;
  auto retreat_poses = get_retreat_poses(breadcrumbs);
  if (!retreat_poses.empty()) {
    send_goal(retreat_poses);
    char buf[128];
    std::snprintf(buf, sizeof(buf),
                  "retreating %.0fm toward better signal",
                  cfg_.retreat_distance_m);
    return buf;
  }

  return advance_to_phase2(breadcrumbs, heatmap_target);
}

std::pair<bool, std::string> RecoveryNavigator::check_phase_complete(
    const std::vector<Breadcrumb>& breadcrumbs,
    const std::optional<geometry_msgs::msg::PoseStamped>& heatmap_target) {
  {
    std::lock_guard<std::mutex> lk(goal_mtx_);
    if (!goal_done_) {
      return {false, current_status_message()};
    }
  }

  if (phase_ == RecoveryPhase::GRADUAL_RETREAT) {
    auto msg = advance_to_phase2(breadcrumbs, heatmap_target);
    return {phase_ == RecoveryPhase::DONE, msg};
  }

  if (phase_ == RecoveryPhase::HEATMAP_GUIDED) {
    auto msg = advance_to_phase3(breadcrumbs);
    return {phase_ == RecoveryPhase::DONE, msg};
  }

  if (phase_ == RecoveryPhase::FULL_BACKTRACK) {
    current_batch_idx_++;
    if (current_batch_idx_ < backtrack_batches_.size()) {
      send_goal(backtrack_batches_[current_batch_idx_]);
      size_t total = 0, remaining = 0;
      for (const auto& b : backtrack_batches_) total += b.size();
      for (size_t i = current_batch_idx_; i < backtrack_batches_.size(); ++i)
        remaining += backtrack_batches_[i].size();
      char buf[128];
      std::snprintf(buf, sizeof(buf),
                    "retracing breadcrumb trail (%zu/%zu)",
                    total - remaining, total);
      return {false, buf};
    }
    phase_ = RecoveryPhase::DONE;
    return {true, "all breadcrumbs exhausted"};
  }

  return {true, "recovery complete"};
}

// -- Internal helpers --

std::string RecoveryNavigator::advance_to_phase2(
    const std::vector<Breadcrumb>& breadcrumbs,
    const std::optional<geometry_msgs::msg::PoseStamped>& heatmap_target) {
  if (heatmap_target) {
    phase_ = RecoveryPhase::HEATMAP_GUIDED;
    send_goal({*heatmap_target});
    return "navigating to nearest known good-signal area";
  }
  return advance_to_phase3(breadcrumbs);
}

std::string RecoveryNavigator::advance_to_phase3(
    const std::vector<Breadcrumb>& breadcrumbs) {
  if (breadcrumbs.empty()) {
    phase_ = RecoveryPhase::DONE;
    return "no breadcrumbs remaining for backtrack";
  }

  // Reverse breadcrumbs (newest first = retrace path)
  std::vector<geometry_msgs::msg::PoseStamped> reversed_poses;
  reversed_poses.reserve(breadcrumbs.size());
  for (auto it = breadcrumbs.rbegin(); it != breadcrumbs.rend(); ++it) {
    reversed_poses.push_back(it->pose);
  }

  // Split into batches
  backtrack_batches_.clear();
  for (size_t i = 0; i < reversed_poses.size();
       i += static_cast<size_t>(cfg_.batch_size)) {
    size_t end =
        std::min(i + static_cast<size_t>(cfg_.batch_size), reversed_poses.size());
    backtrack_batches_.emplace_back(reversed_poses.begin() + static_cast<long>(i),
                                    reversed_poses.begin() + static_cast<long>(end));
  }
  current_batch_idx_ = 0;

  phase_ = RecoveryPhase::FULL_BACKTRACK;
  send_goal(backtrack_batches_[0]);
  char buf[128];
  std::snprintf(buf, sizeof(buf), "retracing breadcrumb trail (0/%zu)",
                reversed_poses.size());
  return buf;
}

std::vector<geometry_msgs::msg::PoseStamped>
RecoveryNavigator::get_retreat_poses(
    const std::vector<Breadcrumb>& breadcrumbs) const {
  if (breadcrumbs.empty()) return {};

  std::vector<geometry_msgs::msg::PoseStamped> poses;
  double cumulative_dist = 0.0;
  const geometry_msgs::msg::PoseStamped* prev = nullptr;

  for (auto it = breadcrumbs.rbegin(); it != breadcrumbs.rend(); ++it) {
    if (prev) {
      double dx = it->pose.pose.position.x - prev->pose.position.x;
      double dy = it->pose.pose.position.y - prev->pose.position.y;
      cumulative_dist += std::hypot(dx, dy);
    }
    poses.push_back(it->pose);
    prev = &(it->pose);

    if (cumulative_dist >= cfg_.retreat_distance_m) break;
  }

  return poses.size() > 1 ? poses : std::vector<geometry_msgs::msg::PoseStamped>{};
}

void RecoveryNavigator::send_goal(
    const std::vector<geometry_msgs::msg::PoseStamped>& poses) {
  {
    std::lock_guard<std::mutex> lk(goal_mtx_);
    goal_done_ = false;
    goal_succeeded_ = false;
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(logger_, "Nav2 action server not available");
    std::lock_guard<std::mutex> lk(goal_mtx_);
    goal_done_ = true;
    goal_succeeded_ = false;
    return;
  }

  auto goal = NavigateThroughPoses::Goal();
  for (const auto& p : poses) {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header.frame_id = cfg_.map_frame;
    stamped.header.stamp = clock_->now();
    stamped.pose = p.pose;
    goal.poses.push_back(stamped);
  }

  RCLCPP_INFO(logger_, "Sending Nav2 goal with %zu poses", goal.poses.size());

  auto send_opts = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
  send_opts.goal_response_callback =
      [this](const GoalHandle::SharedPtr& goal_handle) {
        if (!goal_handle) {
          RCLCPP_WARN(logger_, "Nav2 goal rejected");
          std::lock_guard<std::mutex> lk(goal_mtx_);
          goal_done_ = true;
          goal_succeeded_ = false;
          return;
        }
        {
          std::lock_guard<std::mutex> lk(goal_mtx_);
          goal_handle_ = goal_handle;
        }
      };

  send_opts.result_callback =
      [this](const GoalHandle::WrappedResult& result) {
        std::lock_guard<std::mutex> lk(goal_mtx_);
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(logger_, "Nav2 goal succeeded");
            goal_succeeded_ = true;
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(logger_, "Nav2 goal was cancelled");
            goal_succeeded_ = false;
            break;
          default:
            RCLCPP_WARN(logger_, "Nav2 goal failed");
            goal_succeeded_ = false;
            break;
        }
        goal_done_ = true;
      };

  action_client_->async_send_goal(goal, send_opts);
}

std::string RecoveryNavigator::current_status_message() const {
  switch (phase_) {
    case RecoveryPhase::GRADUAL_RETREAT: {
      char buf[128];
      std::snprintf(buf, sizeof(buf),
                    "retreating %.0fm toward better signal",
                    cfg_.retreat_distance_m);
      return buf;
    }
    case RecoveryPhase::HEATMAP_GUIDED:
      return "navigating to nearest known good-signal area";
    case RecoveryPhase::FULL_BACKTRACK: {
      if (!backtrack_batches_.empty()) {
        size_t total = 0, done = 0;
        for (const auto& b : backtrack_batches_) total += b.size();
        for (size_t i = 0; i < current_batch_idx_ && i < backtrack_batches_.size();
             ++i)
          done += backtrack_batches_[i].size();
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "retracing breadcrumb trail (%zu/%zu)", done, total);
        return buf;
      }
      return "retracing breadcrumb trail";
    }
    default:
      return "recovering";
  }
}

}  // namespace network_recovery

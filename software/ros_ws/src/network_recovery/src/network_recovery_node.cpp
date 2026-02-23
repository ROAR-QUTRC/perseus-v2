#include "network_recovery/network_recovery_node.hpp"

#include <chrono>
#include <cmath>

#include <tf2/exceptions.h>

using namespace std::chrono_literals;

namespace network_recovery {

static const char* state_name(State s) {
  switch (s) {
    case MONITORING: return "MONITORING";
    case SIGNAL_DEGRADED: return "SIGNAL_DEGRADED";
    case SIGNAL_LEASH: return "SIGNAL_LEASH";
    case SIGNAL_LOST_WAITING: return "SIGNAL_LOST_WAITING";
    case RECOVERING: return "RECOVERING";
    case RECOVERY_SUCCEEDED: return "RECOVERY_SUCCEEDED";
    case RECOVERY_FAILED: return "RECOVERY_FAILED";
    default: return "UNKNOWN";
  }
}

static double monotonic_now() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// ─────────────────── Constructor / destructor ───────────────────

NetworkRecoveryNode::NetworkRecoveryNode()
    : Node("network_recovery") {
  declare_params();
  p_ = load_params();

  // QoS
  auto transient_qos = rclcpp::QoS(1)
                            .transient_local()
                            .reliable();

  // Publishers
  status_pub_ = create_publisher<perseus_interfaces::msg::NetworkRecoveryStatus>(
      "/network_recovery/status", transient_qos);
  heatmap_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/network_recovery/heatmap", transient_qos);
  trail_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/network_recovery/breadcrumb_trail", transient_qos);

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Network monitor
  NetworkMonitor::Config mon_cfg;
  mon_cfg.ping_target = p_.ping_target;
  mon_cfg.ping_interval_s = p_.ping_interval_s;
  mon_cfg.ping_timeout_s = p_.ping_timeout_s;
  mon_cfg.wifi_interface = p_.wifi_interface;
  mon_cfg.trend_window_size = p_.trend_window_size;
  mon_cfg.trend_threshold = p_.trend_threshold;
  monitor_ = std::make_unique<NetworkMonitor>(mon_cfg, get_logger());

  // Heatmap
  HeatmapGrid::Config hm_cfg;
  hm_cfg.cell_size_m = p_.heatmap_cell_size_m;
  hm_cfg.marker_height = p_.heatmap_marker_height;
  hm_cfg.marker_z = p_.heatmap_marker_z;
  hm_cfg.marker_alpha = p_.heatmap_marker_alpha;
  hm_cfg.good_signal_threshold = p_.heatmap_good_signal_threshold;
  hm_cfg.map_frame = p_.map_frame;
  heatmap_ = std::make_unique<HeatmapGrid>(hm_cfg);

  // Recovery navigator
  RecoveryNavigator::Config nav_cfg;
  nav_cfg.action_name = p_.nav2_action_name;
  nav_cfg.batch_size = p_.recovery_batch_size;
  nav_cfg.retreat_distance_m = p_.retreat_distance_m;
  nav_cfg.map_frame = p_.map_frame;
  navigator_ = std::make_shared<RecoveryNavigator>(
      get_node_base_interface(),
      get_node_graph_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      get_clock(),
      nav_cfg);

  // Timers
  pose_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / p_.pose_sample_rate_hz),
      std::bind(&NetworkRecoveryNode::pose_sample_cb, this));
  breadcrumb_timer_ = create_wall_timer(
      std::chrono::duration<double>(p_.breadcrumb_interval_s),
      std::bind(&NetworkRecoveryNode::breadcrumb_cb, this));
  status_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / p_.status_publish_rate_hz),
      std::bind(&NetworkRecoveryNode::status_publish_cb, this));
  heatmap_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / p_.heatmap_publish_rate_hz),
      std::bind(&NetworkRecoveryNode::heatmap_publish_cb, this));
  state_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / p_.state_machine_rate_hz),
      std::bind(&NetworkRecoveryNode::state_machine_tick, this));

  monitor_->start();
  RCLCPP_INFO(get_logger(), "NetworkRecoveryNode started");
}

NetworkRecoveryNode::~NetworkRecoveryNode() {
  monitor_->stop();
  navigator_->reset();
  if (p_.heatmap_save_on_shutdown) {
    heatmap_->save_to_disk(p_.heatmap_save_directory, get_logger());
  }
}

// ─────────────────── Parameters ───────────────────

void NetworkRecoveryNode::declare_params() {
  declare_parameter("ping_target", "");
  declare_parameter("ping_interval_s", 2.0);
  declare_parameter("ping_timeout_s", 1.5);
  declare_parameter("wifi_interface", "");

  declare_parameter("rssi_warning_threshold", -70);
  declare_parameter("rssi_critical_threshold", -85);

  declare_parameter("trend_window_size", 10);
  declare_parameter("trend_threshold", 1.0);

  declare_parameter("leash_enabled", true);
  declare_parameter("leash_rssi_threshold", -70);
  declare_parameter("leash_min_samples", 5);
  declare_parameter("leash_pause_robot", false);
  declare_parameter("leash_timeout_s", 60.0);

  declare_parameter("signal_loss_timeout_s", 30.0);
  declare_parameter("recovery_enabled", true);

  declare_parameter("breadcrumb_max_count", 500);
  declare_parameter("breadcrumb_min_distance_m", 0.5);
  declare_parameter("breadcrumb_interval_s", 5.0);
  declare_parameter("breadcrumb_pin_rssi_threshold", -55);
  declare_parameter("pinned_breadcrumb_max_count", 50);
  declare_parameter("breadcrumb_strong_signal_threshold", -55);
  declare_parameter("breadcrumb_strong_distance_m", 2.0);
  declare_parameter("breadcrumb_weak_signal_threshold", -70);
  declare_parameter("breadcrumb_weak_distance_m", 0.3);

  declare_parameter("heatmap_cell_size_m", 0.5);
  declare_parameter("heatmap_publish_rate_hz", 0.5);
  declare_parameter("heatmap_marker_height", 0.1);
  declare_parameter("heatmap_marker_z", 0.05);
  declare_parameter("heatmap_marker_alpha", 0.6);
  declare_parameter("heatmap_good_signal_threshold", -55);

  declare_parameter("heatmap_save_on_shutdown", true);
  declare_parameter("heatmap_save_directory",
                    std::string("~/.ros/network_recovery"));

  declare_parameter("recovery_batch_size", 3);
  declare_parameter("retreat_distance_m", 3.0);
  declare_parameter("nav2_action_name",
                    std::string("/navigate_through_poses"));

  declare_parameter("pose_sample_rate_hz", 2.0);
  declare_parameter("status_publish_rate_hz", 1.0);
  declare_parameter("state_machine_rate_hz", 2.0);

  declare_parameter("map_frame", std::string("map"));
  declare_parameter("base_frame", std::string("base_link"));
}

NetworkRecoveryNode::Params NetworkRecoveryNode::load_params() {
  Params p;
  p.ping_target = get_parameter("ping_target").as_string();
  p.ping_interval_s = get_parameter("ping_interval_s").as_double();
  p.ping_timeout_s = get_parameter("ping_timeout_s").as_double();
  p.wifi_interface = get_parameter("wifi_interface").as_string();

  p.rssi_warning_threshold = static_cast<int>(get_parameter("rssi_warning_threshold").as_int());
  p.rssi_critical_threshold = static_cast<int>(get_parameter("rssi_critical_threshold").as_int());

  p.trend_window_size = static_cast<int>(get_parameter("trend_window_size").as_int());
  p.trend_threshold = get_parameter("trend_threshold").as_double();

  p.leash_enabled = get_parameter("leash_enabled").as_bool();
  p.leash_rssi_threshold = static_cast<int>(get_parameter("leash_rssi_threshold").as_int());
  p.leash_min_samples = static_cast<int>(get_parameter("leash_min_samples").as_int());
  p.leash_pause_robot = get_parameter("leash_pause_robot").as_bool();
  p.leash_timeout_s = get_parameter("leash_timeout_s").as_double();

  p.signal_loss_timeout_s = get_parameter("signal_loss_timeout_s").as_double();
  p.recovery_enabled = get_parameter("recovery_enabled").as_bool();

  p.breadcrumb_max_count = static_cast<int>(get_parameter("breadcrumb_max_count").as_int());
  p.breadcrumb_min_distance_m = get_parameter("breadcrumb_min_distance_m").as_double();
  p.breadcrumb_interval_s = get_parameter("breadcrumb_interval_s").as_double();
  p.breadcrumb_pin_rssi_threshold = static_cast<int>(get_parameter("breadcrumb_pin_rssi_threshold").as_int());
  p.pinned_breadcrumb_max_count = static_cast<int>(get_parameter("pinned_breadcrumb_max_count").as_int());
  p.breadcrumb_strong_signal_threshold = static_cast<int>(get_parameter("breadcrumb_strong_signal_threshold").as_int());
  p.breadcrumb_strong_distance_m = get_parameter("breadcrumb_strong_distance_m").as_double();
  p.breadcrumb_weak_signal_threshold = static_cast<int>(get_parameter("breadcrumb_weak_signal_threshold").as_int());
  p.breadcrumb_weak_distance_m = get_parameter("breadcrumb_weak_distance_m").as_double();

  p.heatmap_cell_size_m = get_parameter("heatmap_cell_size_m").as_double();
  p.heatmap_publish_rate_hz = get_parameter("heatmap_publish_rate_hz").as_double();
  p.heatmap_marker_height = get_parameter("heatmap_marker_height").as_double();
  p.heatmap_marker_z = get_parameter("heatmap_marker_z").as_double();
  p.heatmap_marker_alpha = get_parameter("heatmap_marker_alpha").as_double();
  p.heatmap_good_signal_threshold = static_cast<int>(get_parameter("heatmap_good_signal_threshold").as_int());

  p.heatmap_save_on_shutdown = get_parameter("heatmap_save_on_shutdown").as_bool();
  p.heatmap_save_directory = get_parameter("heatmap_save_directory").as_string();

  p.recovery_batch_size = static_cast<int>(get_parameter("recovery_batch_size").as_int());
  p.retreat_distance_m = get_parameter("retreat_distance_m").as_double();
  p.nav2_action_name = get_parameter("nav2_action_name").as_string();

  p.pose_sample_rate_hz = get_parameter("pose_sample_rate_hz").as_double();
  p.status_publish_rate_hz = get_parameter("status_publish_rate_hz").as_double();
  p.state_machine_rate_hz = get_parameter("state_machine_rate_hz").as_double();

  p.map_frame = get_parameter("map_frame").as_string();
  p.base_frame = get_parameter("base_frame").as_string();

  return p;
}

// ─────────────────── TF lookup ───────────────────

std::optional<geometry_msgs::msg::PoseStamped>
NetworkRecoveryNode::get_current_pose() {
  try {
    auto t = tf_buffer_->lookupTransform(
        p_.map_frame, p_.base_frame, tf2::TimePointZero,
        tf2::durationFromSec(0.1));
    geometry_msgs::msg::PoseStamped pose;
    pose.header = t.header;
    pose.pose.position.x = t.transform.translation.x;
    pose.pose.position.y = t.transform.translation.y;
    pose.pose.position.z = t.transform.translation.z;
    pose.pose.orientation = t.transform.rotation;
    return pose;
  } catch (const tf2::TransformException&) {
    return std::nullopt;
  }
}

// ─────────────────── Timer callbacks ───────────────────

void NetworkRecoveryNode::pose_sample_cb() {
  int rssi = monitor_->rssi_dbm();
  if (rssi == 0) return;
  auto pose = get_current_pose();
  if (!pose) return;
  heatmap_->record(pose->pose.position.x, pose->pose.position.y, rssi);
}

void NetworkRecoveryNode::breadcrumb_cb() {
  if (state_ != MONITORING && state_ != SIGNAL_DEGRADED &&
      state_ != SIGNAL_LEASH)
    return;

  auto pose = get_current_pose();
  if (!pose) return;

  int rssi = monitor_->rssi_dbm();

  // Adaptive spacing
  double min_dist = p_.breadcrumb_min_distance_m;
  if (rssi != 0) {
    if (rssi > p_.breadcrumb_strong_signal_threshold)
      min_dist = p_.breadcrumb_strong_distance_m;
    else if (rssi < p_.breadcrumb_weak_signal_threshold)
      min_dist = p_.breadcrumb_weak_distance_m;
  }

  // Check distance from last breadcrumb
  if (last_breadcrumb_pose_) {
    double dx = pose->pose.position.x - last_breadcrumb_pose_->pose.position.x;
    double dy = pose->pose.position.y - last_breadcrumb_pose_->pose.position.y;
    if (std::hypot(dx, dy) < min_dist) return;
  }

  // Record breadcrumb
  breadcrumbs_.push_back({*pose, rssi});
  while (static_cast<int>(breadcrumbs_.size()) > p_.breadcrumb_max_count) {
    breadcrumbs_.pop_front();
  }
  last_breadcrumb_pose_ = *pose;
  trail_dirty_ = true;

  // Pin if signal is good
  if (rssi != 0 && rssi >= p_.breadcrumb_pin_rssi_threshold) {
    if (static_cast<int>(pinned_breadcrumbs_.size()) >=
        p_.pinned_breadcrumb_max_count) {
      pinned_breadcrumbs_.erase(pinned_breadcrumbs_.begin());
    }
    pinned_breadcrumbs_.push_back({*pose, rssi});
  }
}

void NetworkRecoveryNode::status_publish_cb() {
  perseus_interfaces::msg::NetworkRecoveryStatus msg;
  msg.stamp = now();
  msg.state = state_;
  msg.status_message = build_status_message();
  msg.signal_rssi_dbm = static_cast<int8_t>(monitor_->rssi_dbm());
  msg.ping_latency_ms = monitor_->ping_latency_ms();
  msg.connected = monitor_->connected();
  msg.signal_trend = monitor_->signal_trend();
  msg.signal_trend_rate = monitor_->signal_trend_rate();
  msg.breadcrumbs_total = static_cast<uint16_t>(
      breadcrumbs_.size() + pinned_breadcrumbs_.size());
  msg.breadcrumbs_remaining = static_cast<uint16_t>(breadcrumbs_.size());

  double last_success = monitor_->last_success_monotonic();
  msg.seconds_since_last_connected =
      (last_success > 0.0) ? (monotonic_now() - last_success) : -1.0;

  auto pose = get_current_pose();
  if (pose) msg.current_pose = *pose;

  status_pub_->publish(msg);

  if (trail_dirty_) {
    publish_trail();
    trail_dirty_ = false;
  }
}

void NetworkRecoveryNode::heatmap_publish_cb() {
  auto stamp = now();
  auto ma = heatmap_->to_marker_array(stamp);
  heatmap_pub_->publish(ma);
}

// ─────────────────── State machine ───────────────────

void NetworkRecoveryNode::state_machine_tick() {
  bool connected = monitor_->connected();
  int rssi = monitor_->rssi_dbm();
  auto trend = monitor_->signal_trend();
  double now_mono = monotonic_now();

  auto prev = state_;

  switch (state_) {
    case MONITORING:
      tick_monitoring(connected, rssi, trend, now_mono);
      break;
    case SIGNAL_DEGRADED:
      tick_degraded(connected, rssi, trend, now_mono);
      break;
    case SIGNAL_LEASH:
      tick_leash(connected, rssi, trend, now_mono);
      break;
    case SIGNAL_LOST_WAITING:
      tick_lost_waiting(connected, now_mono);
      break;
    case RECOVERING:
      tick_recovering(connected);
      break;
    case RECOVERY_SUCCEEDED:
      tick_succeeded(now_mono);
      break;
    case RECOVERY_FAILED:
      tick_failed(connected);
      break;
  }

  if (state_ != prev) {
    RCLCPP_INFO(get_logger(), "State: %s -> %s",
                state_name(prev), state_name(state_));
  }
}

void NetworkRecoveryNode::tick_monitoring(bool connected, int rssi,
                                          SignalTrend trend, double now_mono) {
  if (!connected) {
    state_ = SIGNAL_LOST_WAITING;
    signal_lost_time_ = now_mono;
    return;
  }

  if (rssi != 0 && rssi < p_.rssi_warning_threshold) {
    if (p_.leash_enabled && trend == TREND_DEGRADING &&
        rssi < p_.leash_rssi_threshold) {
      leash_degrading_count_++;
      if (leash_degrading_count_ >= p_.leash_min_samples) {
        state_ = SIGNAL_LEASH;
        leash_enter_time_ = now_mono;
        if (p_.leash_pause_robot) navigator_->cancel_all_goals();
        return;
      }
    } else {
      leash_degrading_count_ = 0;
    }
    state_ = SIGNAL_DEGRADED;
    return;
  }
  leash_degrading_count_ = 0;
}

void NetworkRecoveryNode::tick_degraded(bool connected, int rssi,
                                        SignalTrend trend, double now_mono) {
  if (!connected) {
    state_ = SIGNAL_LOST_WAITING;
    signal_lost_time_ = now_mono;
    return;
  }
  if (rssi == 0 || rssi >= p_.rssi_warning_threshold) {
    state_ = MONITORING;
    leash_degrading_count_ = 0;
    return;
  }
  if (p_.leash_enabled && trend == TREND_DEGRADING &&
      rssi < p_.leash_rssi_threshold) {
    leash_degrading_count_++;
    if (leash_degrading_count_ >= p_.leash_min_samples) {
      state_ = SIGNAL_LEASH;
      leash_enter_time_ = now_mono;
      if (p_.leash_pause_robot) navigator_->cancel_all_goals();
    }
  } else {
    leash_degrading_count_ = 0;
  }
}

void NetworkRecoveryNode::tick_leash(bool connected, int rssi,
                                      SignalTrend trend, double now_mono) {
  if (!connected) {
    state_ = SIGNAL_LOST_WAITING;
    signal_lost_time_ = now_mono;
    leash_degrading_count_ = 0;
    return;
  }
  if (trend != TREND_DEGRADING || rssi >= p_.leash_rssi_threshold) {
    state_ = MONITORING;
    leash_degrading_count_ = 0;
    return;
  }
  if (leash_enter_time_ > 0 &&
      (now_mono - leash_enter_time_) >= p_.leash_timeout_s) {
    state_ = SIGNAL_LOST_WAITING;
    signal_lost_time_ = now_mono;
    leash_degrading_count_ = 0;
  }
}

void NetworkRecoveryNode::tick_lost_waiting(bool connected, double now_mono) {
  if (connected) {
    state_ = MONITORING;
    signal_lost_time_ = 0.0;
    leash_degrading_count_ = 0;
    return;
  }
  double elapsed = now_mono - signal_lost_time_;
  if (elapsed >= p_.signal_loss_timeout_s) {
    if (p_.recovery_enabled) {
      begin_recovery();
    } else {
      state_ = RECOVERY_FAILED;
      recovery_status_msg_ = "recovery disabled by configuration";
    }
  }
}

void NetworkRecoveryNode::tick_recovering(bool connected) {
  if (connected) {
    navigator_->cancel_all_goals();
    state_ = RECOVERY_SUCCEEDED;
    recovery_succeeded_time_ = monotonic_now();
    recovery_status_msg_ = "signal reacquired, resuming normal operation";
    return;
  }

  // Build combined breadcrumb list
  std::vector<Breadcrumb> all;
  all.reserve(breadcrumbs_.size() + pinned_breadcrumbs_.size());
  all.insert(all.end(), breadcrumbs_.begin(), breadcrumbs_.end());
  all.insert(all.end(), pinned_breadcrumbs_.begin(),
             pinned_breadcrumbs_.end());

  std::optional<geometry_msgs::msg::PoseStamped> heatmap_target;
  auto pose = get_current_pose();
  if (pose) {
    heatmap_target = heatmap_->find_nearest_good_signal(
        pose->pose.position.x, pose->pose.position.y);
  }

  auto [all_done, msg] =
      navigator_->check_phase_complete(all, heatmap_target);
  recovery_status_msg_ = msg;

  if (all_done) {
    state_ = RECOVERY_FAILED;
    recovery_status_msg_ = "recovery failed — " + msg;
  }
}

void NetworkRecoveryNode::tick_succeeded(double now_mono) {
  if (recovery_succeeded_time_ > 0 &&
      (now_mono - recovery_succeeded_time_) >= 3.0) {
    state_ = MONITORING;
    recovery_succeeded_time_ = 0.0;
    breadcrumbs_.clear();
    last_breadcrumb_pose_.reset();
    trail_dirty_ = true;
    leash_degrading_count_ = 0;
  }
}

void NetworkRecoveryNode::tick_failed(bool connected) {
  if (connected) {
    state_ = MONITORING;
    leash_degrading_count_ = 0;
  }
}

// ─────────────────── Recovery ───────────────────

void NetworkRecoveryNode::begin_recovery() {
  state_ = RECOVERING;

  std::vector<Breadcrumb> all;
  all.reserve(breadcrumbs_.size() + pinned_breadcrumbs_.size());
  all.insert(all.end(), breadcrumbs_.begin(), breadcrumbs_.end());
  all.insert(all.end(), pinned_breadcrumbs_.begin(),
             pinned_breadcrumbs_.end());

  std::optional<geometry_msgs::msg::PoseStamped> heatmap_target;
  auto pose = get_current_pose();
  if (pose) {
    heatmap_target = heatmap_->find_nearest_good_signal(
        pose->pose.position.x, pose->pose.position.y);
  }

  recovery_status_msg_ = navigator_->start_recovery(all, heatmap_target);

  if (navigator_->phase() == RecoveryPhase::DONE) {
    state_ = RECOVERY_FAILED;
    recovery_status_msg_ = "recovery failed — " + recovery_status_msg_;
  }
}

// ─────────────────── Status messages ───────────────────

std::string NetworkRecoveryNode::build_status_message() {
  int rssi = monitor_->rssi_dbm();
  float latency = monitor_->ping_latency_ms();
  bool connected = monitor_->connected();
  char buf[256];

  switch (state_) {
    case MONITORING:
      if (rssi != 0 && connected) {
        std::snprintf(buf, sizeof(buf),
                      "normal (rssi: %d dBm, latency: %.0fms)",
                      rssi, latency);
        return buf;
      }
      return connected ? "normal" : "normal (no RSSI data)";

    case SIGNAL_DEGRADED:
      std::snprintf(buf, sizeof(buf), "signal degraded (rssi: %d dBm)", rssi);
      return buf;

    case SIGNAL_LEASH: {
      float rate = monitor_->signal_trend_rate();
      std::snprintf(buf, sizeof(buf),
                    "signal leash — trend degrading, %s "
                    "(rssi: %d dBm, trend: %.1f dBm/sample)",
                    p_.leash_pause_robot ? "pausing" : "warning",
                    rssi, rate);
      return buf;
    }

    case SIGNAL_LOST_WAITING: {
      double elapsed = monotonic_now() - signal_lost_time_;
      std::snprintf(buf, sizeof(buf),
                    "loss of signal, waiting (%.0fs / %.0fs)",
                    elapsed, p_.signal_loss_timeout_s);
      return buf;
    }

    case RECOVERING:
      return "confirmed signal lost, " + recovery_status_msg_;

    case RECOVERY_SUCCEEDED:
      return recovery_status_msg_;

    case RECOVERY_FAILED:
      return recovery_status_msg_.empty()
                 ? "recovery failed — awaiting operator"
                 : recovery_status_msg_;
  }
  return "unknown state";
}

// ─────────────────── Trail ───────────────────

void NetworkRecoveryNode::publish_trail() {
  nav_msgs::msg::Path path;
  path.header.frame_id = p_.map_frame;
  path.header.stamp = now();
  for (const auto& bc : breadcrumbs_) {
    path.poses.push_back(bc.pose);
  }
  trail_pub_->publish(path);
}

}  // namespace network_recovery

// ─────────────────── main ───────────────────

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<network_recovery::NetworkRecoveryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

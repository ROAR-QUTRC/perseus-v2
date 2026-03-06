#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "domain_bridge/domain_bridge.hpp"
#include "domain_bridge/domain_bridge_options.hpp"
#include "domain_bridge/topic_bridge_options.hpp"

namespace domain_bridge_throttled
{

/// Configuration for a single bridged topic.
struct TopicConfig
{
  std::string topic;        ///< Source topic name
  std::string type;         ///< ROS message type, e.g. "sensor_msgs/msg/Image"
  std::string remap;        ///< Destination topic name (empty = same as source)
  std::string qos;          ///< "reliable" | "best_effort"
  double max_rate{0.0};     ///< Hz limit (0 = unlimited / full passthrough)
  int from_domain{-1};      ///< -1 = use global default
  int to_domain{-1};        ///< -1 = use global default
};

/// Global bridge configuration loaded from YAML.
struct BridgeConfig
{
  int global_from_domain{0};
  int global_to_domain{1};
  std::vector<TopicConfig> topics;
};

// ──────────────────────────────────────────────────────────────────────────
// ThrottledGenericBridge
//
// For a single topic, owns:
//   • A rclcpp::GenericSubscription on `from_domain`
//   • A rclcpp::GenericPublisher   on `to_domain`
//
// The subscriber callback gates publication via a wall-clock interval check
// (monotonic_clock), dropping messages that arrive too soon.  No queue,
// no timer – minimal latency overhead.
// ──────────────────────────────────────────────────────────────────────────
class ThrottledGenericBridge
{
public:
  ThrottledGenericBridge(
    rclcpp::Node::SharedPtr src_node,
    rclcpp::Node::SharedPtr dst_node,
    const TopicConfig & cfg);

private:
  void on_message(std::shared_ptr<rclcpp::SerializedMessage> msg);

  rclcpp::GenericPublisher::SharedPtr publisher_;
  rclcpp::GenericSubscription::SharedPtr subscription_;

  std::chrono::duration<double> min_interval_{0};   ///< 1/max_rate seconds
  std::chrono::steady_clock::time_point last_pub_{};
  std::mutex mutex_;
};

// ──────────────────────────────────────────────────────────────────────────
// ThrottledDomainBridge
//
// Orchestrates N ThrottledGenericBridge instances.
// One rclcpp::Node per unique domain_id; each node is spun by its own
// rclcpp::executors::MultiThreadedExecutor on a dedicated std::thread.
// ──────────────────────────────────────────────────────────────────────────
class ThrottledDomainBridge
{
public:
  explicit ThrottledDomainBridge(const std::string & config_path);
  ~ThrottledDomainBridge();

  /// Build subscriptions/publishers from config, then block until shutdown.
  void run();

private:
  /// Return (creating if necessary) the Node for the given domain.
  rclcpp::Node::SharedPtr get_or_create_node(int domain_id);

  BridgeConfig config_;

  // domain_id  →  context + node + executor + thread
  struct DomainContext
  {
    rclcpp::Context::SharedPtr context;
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
    std::thread spin_thread;
  };

  std::unordered_map<int, DomainContext> domains_;
  std::vector<std::shared_ptr<ThrottledGenericBridge>> bridges_;
};

/// Parse YAML config file → BridgeConfig.
BridgeConfig load_config(const std::string & path);

}  // namespace domain_bridge_throttled
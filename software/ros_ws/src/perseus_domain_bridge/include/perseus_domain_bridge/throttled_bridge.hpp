#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "domain_bridge/domain_bridge.hpp"
#include "domain_bridge/domain_bridge_options.hpp"
#include "domain_bridge/topic_bridge_options.hpp"
#include "rclcpp/rclcpp.hpp"

namespace domain_bridge_throttled
{

    /// Configuration for a single bridged topic.
    struct TopicConfig
    {
        std::string topic;  ///< Source topic name
        std::string type;   ///< ROS message type, e.g. "sensor_msgs/msg/Image"
        std::string remap;  ///< Destination topic name (empty = same as source)

        /// QoS for both subscriber and publisher when sub_qos/pub_qos are not set.
        /// Values: "reliable" | "best_effort" | "transient_local"
        std::string qos{"best_effort"};

        /// Optional: QoS to use only for the subscriber (source domain).
        /// When empty, falls back to `qos`.
        /// Use this when the source publishes best_effort but the destination
        /// consumers require reliable – set sub_qos: best_effort, pub_qos: reliable.
        std::string sub_qos;

        /// Optional: QoS to use only for the publisher (destination domain).
        /// When empty, falls back to `qos`.
        std::string pub_qos;

        double max_rate{0.0};  ///< Hz limit (0 = unlimited / full passthrough)
        int from_domain{-1};   ///< -1 = use global default
        int to_domain{-1};     ///< -1 = use global default
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
    // (steady_clock), dropping messages that arrive too soon.  No queue,
    // no timer – minimal latency overhead.
    //
    // sub_qos and pub_qos can differ, allowing the bridge to subscribe to a
    // best_effort source while publishing as reliable to satisfy downstream
    // consumers (e.g. Nav2 subscribing to /tf as reliable).
    // ──────────────────────────────────────────────────────────────────────────
    class ThrottledGenericBridge
    {
    public:
        ThrottledGenericBridge(
            rclcpp::Node::SharedPtr src_node,
            rclcpp::Node::SharedPtr dst_node,
            const TopicConfig& cfg);

    private:
        void on_message(std::shared_ptr<rclcpp::SerializedMessage> msg);

        rclcpp::GenericPublisher::SharedPtr publisher_;
        rclcpp::GenericSubscription::SharedPtr subscription_;

        std::chrono::duration<double> min_interval_{0};
        std::chrono::steady_clock::time_point last_pub_{};
        std::mutex mutex_;
    };

    // ──────────────────────────────────────────────────────────────────────────
    // ThrottledDomainBridge
    // ──────────────────────────────────────────────────────────────────────────
    class ThrottledDomainBridge
    {
    public:
        explicit ThrottledDomainBridge(const std::string& config_path);
        ~ThrottledDomainBridge();

        void run();

    private:
        rclcpp::Node::SharedPtr get_or_create_node(int domain_id);

        BridgeConfig config_;

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
    BridgeConfig load_config(const std::string& path);

}  // namespace domain_bridge_throttled
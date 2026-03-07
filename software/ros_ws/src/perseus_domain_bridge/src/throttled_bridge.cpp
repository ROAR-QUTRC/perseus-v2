#include "perseus_domain_bridge/throttled_bridge.hpp"

#include <fstream>
#include <stdexcept>
#include <string>

#include "rclcpp/qos.hpp"
#include "rclcpp/serialized_message.hpp"
#include "yaml-cpp/yaml.h"

namespace domain_bridge_throttled
{

// ══════════════════════════════════════════════════════════════════════════
// Config loader
// ══════════════════════════════════════════════════════════════════════════

BridgeConfig load_config(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to open config file '" + path + "': " + e.what());
  }

  BridgeConfig cfg;

  if (root["from_domain"]) {cfg.global_from_domain = root["from_domain"].as<int>();}
  if (root["to_domain"])   {cfg.global_to_domain   = root["to_domain"].as<int>();}

  const auto & topics_node = root["topics"];
  if (!topics_node || !topics_node.IsMap()) {
    throw std::runtime_error("Config must contain a 'topics' map.");
  }

  for (const auto & kv : topics_node) {
    TopicConfig tc;
    tc.topic = kv.first.as<std::string>();

    const auto & opts = kv.second;
    if (!opts.IsMap()) {
      RCLCPP_WARN(rclcpp::get_logger("load_config"),
        "Skipping malformed topic entry: %s", tc.topic.c_str());
      continue;
    }
    if (!opts["type"]) {
      RCLCPP_WARN(rclcpp::get_logger("load_config"),
        "No 'type' for topic %s, skipping.", tc.topic.c_str());
      continue;
    }

    tc.type        = opts["type"].as<std::string>();
    tc.max_rate    = opts["max_rate"]    ? opts["max_rate"].as<double>()    : 0.0;
    tc.qos         = opts["qos"]         ? opts["qos"].as<std::string>()    : "best_effort";
    tc.sub_qos     = opts["sub_qos"]     ? opts["sub_qos"].as<std::string>(): "";
    tc.pub_qos     = opts["pub_qos"]     ? opts["pub_qos"].as<std::string>(): "";
    tc.remap       = opts["remap"]       ? opts["remap"].as<std::string>()  : "";
    tc.from_domain = opts["from_domain"] ? opts["from_domain"].as<int>()    : -1;
    tc.to_domain   = opts["to_domain"]   ? opts["to_domain"].as<int>()      : -1;

    cfg.topics.push_back(std::move(tc));
  }

  return cfg;
}

// ══════════════════════════════════════════════════════════════════════════
// QoS helper
// ══════════════════════════════════════════════════════════════════════════

static rclcpp::QoS make_qos(const std::string & qos_str, int depth = 10)
{
  // "transient_local" = reliable + transient_local durability.
  // Required for latched topics: /tf_static, /robot_description, /map.
  if (qos_str == "transient_local") {
    rclcpp::QoS qos(depth);
    qos.reliable();
    qos.transient_local();
    return qos;
  }

  rclcpp::QoS qos(depth);
  if (qos_str == "reliable") {
    qos.reliable();
    qos.durability_volatile();
  } else {
    // default: best_effort + volatile
    qos.best_effort();
    qos.durability_volatile();
  }
  return qos;
}

// ══════════════════════════════════════════════════════════════════════════
// ThrottledGenericBridge
// ══════════════════════════════════════════════════════════════════════════

ThrottledGenericBridge::ThrottledGenericBridge(
  rclcpp::Node::SharedPtr src_node,
  rclcpp::Node::SharedPtr dst_node,
  const TopicConfig & cfg)
{
  const std::string dst_topic = cfg.remap.empty() ? cfg.topic : cfg.remap;

  if (cfg.max_rate > 0.0) {
    min_interval_ = std::chrono::duration<double>(1.0 / cfg.max_rate);
  }
  last_pub_ = std::chrono::steady_clock::now() - std::chrono::hours(1);

  // Resolve sub/pub QoS independently.
  // sub_qos / pub_qos override the shared `qos` field when set.
  const std::string sub_qos_str = cfg.sub_qos.empty() ? cfg.qos : cfg.sub_qos;
  const std::string pub_qos_str = cfg.pub_qos.empty() ? cfg.qos : cfg.pub_qos;

  auto sub_qos = make_qos(sub_qos_str);
  auto pub_qos = make_qos(pub_qos_str);

  const std::string rate_info =
    (cfg.max_rate > 0.0) ? (std::to_string(static_cast<int>(cfg.max_rate)) + " Hz") : "unlimited";

  RCLCPP_INFO(src_node->get_logger(),
    "[Bridge] %s → %s  |  sub_qos: %s  pub_qos: %s  |  rate: %s",
    cfg.topic.c_str(), dst_topic.c_str(),
    sub_qos_str.c_str(), pub_qos_str.c_str(),
    rate_info.c_str());

  publisher_ = dst_node->create_generic_publisher(dst_topic, cfg.type, pub_qos);

  subscription_ = src_node->create_generic_subscription(
    cfg.topic, cfg.type, sub_qos,
    [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      this->on_message(std::move(msg));
    });
}

void ThrottledGenericBridge::on_message(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  if (min_interval_.count() > 0.0) {
    auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if ((now - last_pub_) < min_interval_) {return;}
    last_pub_ = now;
  }
  publisher_->publish(*msg);
}

// ══════════════════════════════════════════════════════════════════════════
// ThrottledDomainBridge
// ══════════════════════════════════════════════════════════════════════════

ThrottledDomainBridge::ThrottledDomainBridge(const std::string & config_path)
: config_(load_config(config_path)) {}

ThrottledDomainBridge::~ThrottledDomainBridge()
{
  for (auto & [id, dc] : domains_) {dc.executor->cancel();}
  for (auto & [id, dc] : domains_) {
    if (dc.spin_thread.joinable()) {dc.spin_thread.join();}
  }
  for (auto & [id, dc] : domains_) {rclcpp::shutdown(dc.context);}
}

rclcpp::Node::SharedPtr ThrottledDomainBridge::get_or_create_node(int domain_id)
{
  auto it = domains_.find(domain_id);
  if (it != domains_.end()) {return it->second.node;}

  DomainContext dc;
  rclcpp::InitOptions init_opts;
  init_opts.set_domain_id(static_cast<size_t>(domain_id));

  dc.context = std::make_shared<rclcpp::Context>();
  dc.context->init(0, nullptr, init_opts);

  rclcpp::NodeOptions node_opts;
  node_opts.context(dc.context);

  const std::string node_name = "throttled_bridge_d" + std::to_string(domain_id);
  dc.node = rclcpp::Node::make_shared(node_name, node_opts);

  rclcpp::ExecutorOptions exec_opts;
  exec_opts.context = dc.context;
  dc.executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(exec_opts);
  dc.executor->add_node(dc.node);

  RCLCPP_INFO(dc.node->get_logger(),
    "Created node '%s' on domain %d", node_name.c_str(), domain_id);

  dc.spin_thread = std::thread([exec = dc.executor]() {exec->spin();});

  domains_.emplace(domain_id, std::move(dc));
  return domains_.at(domain_id).node;
}

void ThrottledDomainBridge::run()
{
  for (const auto & tc : config_.topics) {
    const int from = (tc.from_domain >= 0) ? tc.from_domain : config_.global_from_domain;
    const int to   = (tc.to_domain   >= 0) ? tc.to_domain   : config_.global_to_domain;
    get_or_create_node(from);
    get_or_create_node(to);
  }

  for (const auto & tc : config_.topics) {
    const int from = (tc.from_domain >= 0) ? tc.from_domain : config_.global_from_domain;
    const int to   = (tc.to_domain   >= 0) ? tc.to_domain   : config_.global_to_domain;
    bridges_.push_back(std::make_shared<ThrottledGenericBridge>(
      domains_.at(from).node, domains_.at(to).node, tc));
  }

  RCLCPP_INFO(rclcpp::get_logger("ThrottledDomainBridge"),
    "Running %zu topic bridge(s). Press Ctrl-C to stop.", bridges_.size());

  rclcpp::on_shutdown([]() {
    RCLCPP_INFO(rclcpp::get_logger("ThrottledDomainBridge"), "Shutting down.");
  });

  while (true) {
    bool all_ok = true;
    for (auto & [id, dc] : domains_) {
      if (!dc.context->is_valid()) {all_ok = false; break;}
    }
    if (!all_ok) {break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

}  // namespace domain_bridge_throttled
/**
 * test_talker.cpp
 * ---------------
 * Publishes std_msgs/msg/String on a given topic at a given rate.
 * Designed to run on a specific DDS domain to test the throttled bridge.
 *
 * CLI args:
 *   test_talker <domain_id> <topic> <publish_hz>
 *
 * Example:
 *   ros2 run domain_bridge_throttled test_talker 0 /chatter 60.0
 */

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char ** argv)
{
  if (argc < 4) {
    fprintf(
      stderr,
      "Usage: test_talker <domain_id> <topic> <publish_hz>\n"
      "  e.g. test_talker 0 /chatter 60.0\n");
    return 1;
  }

  const int domain_id    = std::stoi(argv[1]);
  const std::string topic = argv[2];
  const double pub_hz    = std::stod(argv[3]);

  // ── Dedicated context for this domain ──────────────────────────────────
  rclcpp::InitOptions init_opts;
  init_opts.set_domain_id(static_cast<size_t>(domain_id));

  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr, init_opts);

  rclcpp::NodeOptions node_opts;
  node_opts.context(context);

  auto node = rclcpp::Node::make_shared("test_talker", node_opts);

  auto pub = node->create_publisher<std_msgs::msg::String>(topic, 10);

  const auto period = std::chrono::duration<double>(1.0 / pub_hz);
  uint64_t seq = 0;

  auto timer = node->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [&]() {
      std_msgs::msg::String msg;
      msg.data = "[domain " + std::to_string(domain_id) + "] hello #" + std::to_string(seq++);
      pub->publish(msg);
      RCLCPP_INFO_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        1000,   // log at most once per second
        "Publishing @ %.0f Hz on domain %d | %s | msg #%lu",
        pub_hz, domain_id, topic.c_str(), seq - 1);
    });

  RCLCPP_INFO(
    node->get_logger(),
    "test_talker started: domain=%d  topic=%s  rate=%.0f Hz",
    domain_id, topic.c_str(), pub_hz);

  rclcpp::ExecutorOptions exec_opts;
  exec_opts.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(exec_opts);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown(context);
  return 0;
}
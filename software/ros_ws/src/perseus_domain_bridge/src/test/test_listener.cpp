/**
 * test_listener.cpp
 * -----------------
 * Subscribes to std_msgs/msg/String on a given topic and domain.
 * Measures and reports the actual received message rate (Hz) every second,
 * so you can verify the bridge is throttling correctly.
 *
 * CLI args:
 *   test_listener <domain_id> <topic>
 *
 * Example:
 *   ros2 run domain_bridge_throttled test_listener 1 /chatter
 */

#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        fprintf(
            stderr,
            "Usage: test_listener <domain_id> <topic>\n"
            "  e.g. test_listener 1 /chatter\n");
        return 1;
    }

    const int domain_id = std::stoi(argv[1]);
    const std::string topic = argv[2];

    // ── Dedicated context for this domain ──────────────────────────────────
    rclcpp::InitOptions init_opts;
    init_opts.set_domain_id(static_cast<size_t>(domain_id));

    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr, init_opts);

    rclcpp::NodeOptions node_opts;
    node_opts.context(context);

    auto node = rclcpp::Node::make_shared("test_listener", node_opts);

    // ── Rate measurement state ──────────────────────────────────────────────
    std::atomic<uint64_t> msg_count{0};
    std::atomic<uint64_t> total_count{0};
    std::string last_msg;

    auto sub = node->create_subscription<std_msgs::msg::String>(
        topic,
        rclcpp::QoS(10).best_effort(),
        [&](const std_msgs::msg::String::SharedPtr msg)
        {
            ++msg_count;
            ++total_count;
            last_msg = msg->data;
        });

    // ── 1-second report timer ───────────────────────────────────────────────
    auto report_timer = node->create_wall_timer(
        std::chrono::seconds(1),
        [&]()
        {
            uint64_t n = msg_count.exchange(0);
            RCLCPP_INFO(
                node->get_logger(),
                "[domain %d | %s]  received: %lu msg/s  |  total: %lu  |  last: \"%s\"",
                domain_id, topic.c_str(), n, total_count.load(), last_msg.c_str());
        });

    RCLCPP_INFO(
        node->get_logger(),
        "test_listener started: domain=%d  topic=%s  — waiting for messages...",
        domain_id, topic.c_str());

    rclcpp::ExecutorOptions exec_opts;
    exec_opts.context = context;
    rclcpp::executors::SingleThreadedExecutor executor(exec_opts);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown(context);
    return 0;
}
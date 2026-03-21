#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <nav_msgs/msg/path.hpp>
#include <hi_can_raw.hpp>

#include "perseus_lights/light_driver.hpp"  // for ring::commands

class LightStatusOrchestrator : public rclcpp::Node
{
public:
    explicit LightStatusOrchestrator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // --- Callbacks ---
    void _joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void _path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void _can_callback(/* CAN message type */);

    // --- Topic monitoring ---
    void _check_topic_timer_callback();

    // --- State management ---
    void _update_status();
    void _publish_status(ring::commands command);

    // --- Subscriptions ---
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_subscription;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr   _path_subscription;

    // --- Publisher ---
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _status_publisher;

    // --- Timer (for topic-presence detection) ---
    rclcpp::TimerBase::SharedPtr _topic_check_timer;

    // --- State flags ---
    bool _joy_seen      = false;
    bool _map_seen      = false;
    bool _path_received = false;
    bool _power_bus_off = false;
    bool _error_state   = false;

    // --- Timestamp of last joy message (for liveness check) ---
    rclcpp::Time _last_joy_time;
    rclcpp::Time _last_map_time;

    // Timeout threshold (seconds) for considering a topic "gone"
    static constexpr double TOPIC_TIMEOUT_S = 2.0;
};
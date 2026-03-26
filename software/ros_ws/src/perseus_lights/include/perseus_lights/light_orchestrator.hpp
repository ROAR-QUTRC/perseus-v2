#pragma once
#include <hi_can_raw.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int8.hpp>
#include "perseus_interfaces/msg/navigation_data.hpp"
#include "perseus_lights/light_driver.hpp"  // for ring::commands

class LightStatusOrchestrator : public rclcpp::Node
{
public:
    explicit LightStatusOrchestrator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // --- Callbacks ---
    void _joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void _nav_data_callback(const perseus_interfaces::msg::NavigationData::SharedPtr msg);
    void _can_callback(const std::vector<uint8_t>& data);
    void _handle_rcb_status(const hi_can::Packet& packet);

    // --- Topic monitoring ---
    void _check_topic_timer_callback();

    // --- State management ---
    void _update_status();
    void _publish_status(ring::commands command);

    // --- Subscriptions ---
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_subscription;
    rclcpp::Subscription<perseus_interfaces::msg::NavigationData>::SharedPtr _nav_data_subscription;

    // --- CAN ---
    rclcpp::TimerBase::SharedPtr _packet_timer;
    constexpr static auto PACKET_HANLDE_MS = std::chrono::milliseconds(100);
    void _handle_can();

    // --- Publisher ---
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _status_publisher;

    // --- Timer (for topic-presence detection) ---
    rclcpp::TimerBase::SharedPtr _topic_check_timer;

    // --- State flags ---
    bool _joy_seen            = false;
    bool _autonomy_bridge_seen = false;
    bool _navigation_active   = false;
    bool _power_bus_off       = false;
    bool _error_state         = false;

    std::optional<hi_can::RawCanInterface> _can_interface;
    std::optional<hi_can::PacketManager>   _packet_manager;

    // --- Timestamps for liveness checks ---
    rclcpp::Time _last_joy_time;
    rclcpp::Time _last_nav_data_time;

    // Timeout threshold (seconds) for considering a topic "gone"
    static constexpr double TOPIC_TIMEOUT_S = 2.0;
};
#include "perseus_lights/light_orchestrator.hpp"

#include <chrono>

using namespace std::chrono_literals;
using namespace hi_can::addressing;
using namespace hi_can::parameters::legacy::power::control::power_bus;

LightStatusOrchestrator::LightStatusOrchestrator(const rclcpp::NodeOptions& options)
    : Node("light_status_orchestrator", options)
{
    _status_publisher = this->create_publisher<std_msgs::msg::Int8>("light_status", 10);

    // /joy subscription — presence = blue, absence = white
    _joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&LightStatusOrchestrator::_joy_callback, this, std::placeholders::_1));

    // /path subscription — nav2 has given us a path → green
    _path_subscription = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10,
        std::bind(&LightStatusOrchestrator::_path_callback, this, std::placeholders::_1));

    // CAN subscription for power bus status → yellow
    try
    {
        _can_interface.emplace(hi_can::RawCanInterface(this->declare_parameter("can_bus", "can0")));
        _packet_manager.emplace(_can_interface.value());

        _packet_manager->set_callback(
            filter_t{static_cast<flagged_address_t>(legacy::address_t{
                legacy::power::SYSTEM_ID,
                legacy::power::control::SUBSYSTEM_ID,
                static_cast<uint8_t>(legacy::power::control::device::ROVER_CONTROL_BOARD),
                static_cast<uint8_t>(legacy::power::control::rcb::groups::DRIVE_BUS),
                static_cast<uint8_t>(legacy::power::control::power_bus::parameter::POWER_STATUS)})},
            hi_can::PacketManager::callback_config_t{
                .data_callback = std::bind(&LightStatusOrchestrator::_handle_rcb_status, this, std::placeholders::_1),
            });
    }
    catch (const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "Failed to CAN");
    }

    _packet_timer = this->create_wall_timer(
        PACKET_HANDLE_MS,
        std::bind(&LightStatusOrchestrator::_handle_can, this));

    // Timer: periodically check topic liveness (detects dropped topics)
    _topic_check_timer = this->create_wall_timer(
        500ms,
        std::bind(&LightStatusOrchestrator::_check_topic_timer_callback, this));

    _last_joy_time = this->now();
    _last_map_time = this->now();

    RCLCPP_INFO(this->get_logger(), "Light Status Orchestrator node initialised");
}

// ---------------------------------------------------------------------------
// /joy received → topic is alive
// ---------------------------------------------------------------------------
void LightStatusOrchestrator::_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    (void)msg;
    _last_joy_time = this->now();
    bool was_seen = _joy_seen;
    _joy_seen = true;

    if (!was_seen)
    {
        RCLCPP_INFO(this->get_logger(), "[Trigger] /joy topic is now visible");
        _update_status();
    }
}

// ---------------------------------------------------------------------------
// /plan received → nav2 has issued a path
// ---------------------------------------------------------------------------
void LightStatusOrchestrator::_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    (void)msg;
    bool was_received = _path_received;
    _path_received = true;

    if (!was_received)
    {
        RCLCPP_INFO(this->get_logger(), "[Trigger] Nav2 path received");
        _update_status();
    }
}

// ---------------------------------------------------------------------------
// CAN callback → check power bus status → yellow if bus off
// ---------------------------------------------------------------------------
void LightStatusOrchestrator::_handle_can()
{
    try
    {
        _packet_manager->handle();
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN(this->get_logger(), "CAN handle failed");
    }
}

void LightStatusOrchestrator::_handle_rcb_status(const hi_can::Packet& packet)
{
    try
    {
        // Print received packet info
        standard_address_t address{packet.get_address().address};

        LightStatusOrchestrator::_can_callback(packet.get_data());
    }
    catch (const std::exception& e)
    {
    }
}

void LightStatusOrchestrator::_can_callback(const std::vector<uint8_t>& data)
{
    using namespace hi_can::parameters::legacy::power::control::power_bus;

    status_t status;
    status.deserialize_data(data);

    switch (status.status)
    {
    case power_status::OFF:
    {
        _power_bus_off = true;
        _error_state = false;
        break;
    }
    case power_status::FAULT:
    {
        _error_state = true;
        break;
    }
    case power_status::ON:
    {
        _power_bus_off = false;
        _error_state = false;
        break;
    }
    default:
        break;
    }

    RCLCPP_WARN(this->get_logger(), "[Trigger] Power bus changed");

    _update_status();
}

// ---------------------------------------------------------------------------
// Periodic check: detect if topics have gone silent
// ---------------------------------------------------------------------------
void LightStatusOrchestrator::_check_topic_timer_callback()
{
    const auto now = this->now();
    const double joy_age = (now - _last_joy_time).seconds();

    // /joy timed out → topic gone → white
    if (_joy_seen && joy_age > TOPIC_TIMEOUT_S)
    {
        RCLCPP_WARN(this->get_logger(), "[Trigger] /joy topic lost (timeout %.1fs)", joy_age);
        _joy_seen = false;
        _path_received = false;  // path is no longer meaningful without joystick
        _update_status();
    }

    // TODO: similarly check /map liveness if you have a /map subscription
}

// ---------------------------------------------------------------------------
// Priority-ordered status resolution
// Red > Yellow > Green > Cyan > Blue > White
// ---------------------------------------------------------------------------
void LightStatusOrchestrator::_update_status()
{
    ring::commands status;

    if (_error_state)
    {
        RCLCPP_ERROR(this->get_logger(), "[Status] RED — error state active");
        status = ring::commands::RED;
    }
    else if (_power_bus_off)
    {
        RCLCPP_WARN(this->get_logger(), "[Status] YELLOW — power bus off");
        status = ring::commands::YELLOW;
    }
    else if (_path_received)
    {
        RCLCPP_INFO(this->get_logger(), "[Status] GREEN — nav2 path active");
        status = ring::commands::GREEN;
    }
    else if (_map_seen)
    {
        RCLCPP_INFO(this->get_logger(), "[Status] CYAN — /map visible");
        status = ring::commands::CYAN;
    }
    else if (_joy_seen)
    {
        RCLCPP_INFO(this->get_logger(), "[Status] BLUE — /joy visible");
        status = ring::commands::BLUE;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "[Status] WHITE — idle, no joy");
        status = ring::commands::WHITE;
    }

    _publish_status(status);
}

void LightStatusOrchestrator::_publish_status(ring::commands command)
{
    auto msg = std_msgs::msg::Int8();
    msg.data = static_cast<int8_t>(command);
    _status_publisher->publish(msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<LightStatusOrchestrator>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting light status orchestrator");
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running orchestrator: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
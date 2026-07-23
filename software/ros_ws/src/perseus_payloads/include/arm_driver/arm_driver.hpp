#pragma once

#include <actuator_msgs/msg/actuators.hpp>
#include <hi_can_raw.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

class ArmDriver : public rclcpp::Node
{
public:
    explicit ArmDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ArmDriver() override;

    void cleanup();

private:
    using joint_id = hi_can::addressing::post_landing::arm::control_board::joint_id;
    using parameter_group =
        hi_can::parameters::post_landing::arm::control_board::ControlBoardParameterGroup;

    enum class startup_phase
    {
        DISCOVERING,
        ZEROING,
        ENABLING,
        RUNNING,
    };

    struct joint_startup_t
    {
        bool candidate = false;
        bool enable_requested = false;
        bool commandable = false;
        bool failure_reported = false;
    };

    void _handle_arm_control(const actuator_msgs::msg::Actuators::SharedPtr message);
    void _handle_can();
    void _initialize_arm_board();
    void _finish_discovery();
    void _advance_zeroing();
    void _advance_enabling();
    void _finish_startup();
    void _publish_status_messages();
    void _publish_motor_positions();
    void _report_failure(joint_id joint, std::string_view reason);
    void _transmit(const hi_can::Packet& packet);

    static std::size_t _joint_index(joint_id joint);
    static std::string_view _joint_name(joint_id joint);
    static int16_t _position_to_wire(double radians);
    static uint16_t _speed_to_wire(double radians_per_second);
    static uint8_t _acceleration_to_wire(double normalized);
    static double _position_from_wire(int16_t milliradians);
    static double _speed_from_wire(int16_t milliradians_per_second);

    static constexpr auto PACKET_HANDLE_PERIOD = std::chrono::milliseconds(5);
    static constexpr auto STARTUP_PERIOD = std::chrono::milliseconds(20);
    static constexpr auto STATUS_PERIOD = std::chrono::milliseconds(100);
    static constexpr double MILLIRADIANS_PER_RADIAN = 1000.0;
    static constexpr std::size_t JOINT_COUNT =
        hi_can::addressing::post_landing::arm::control_board::ALL_JOINTS.size();

    std::optional<hi_can::RawCanInterface> _can_interface;
    std::optional<hi_can::PacketManager> _packet_manager;
    parameter_group _control_board;

    startup_phase _startup_phase = startup_phase::DISCOVERING;
    std::chrono::steady_clock::time_point _phase_started;
    std::chrono::milliseconds _discovery_timeout{1000};
    std::chrono::milliseconds _zero_timeout{5000};
    std::chrono::milliseconds _enable_timeout{1000};
    std::array<joint_startup_t, JOINT_COUNT> _joint_startup{};
    bool _cleaned_up = false;

    rclcpp::TimerBase::SharedPtr _packet_timer;
    rclcpp::TimerBase::SharedPtr _startup_timer;
    rclcpp::TimerBase::SharedPtr _status_timer;
    rclcpp::TimerBase::SharedPtr _position_timer;

    rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr _control_subscriber;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _status_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _position_publisher;
};

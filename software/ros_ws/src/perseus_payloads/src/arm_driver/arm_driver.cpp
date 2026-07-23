#include "arm_driver/arm_driver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

using hi_can::addressing::post_landing::arm::control_board::ALL_JOINTS;
using hi_can::parameters::post_landing::arm::control_board::joint_command_t;

ArmDriver::ArmDriver(const rclcpp::NodeOptions& options)
    : Node("arm_driver", options),
      _phase_started(std::chrono::steady_clock::now())
{
    const auto can_bus = declare_parameter<std::string>("can_bus", "can0");
    const auto discovery_timeout_ms = declare_parameter<int64_t>("discovery_timeout_ms", 1000);
    const auto zero_timeout_ms = declare_parameter<int64_t>("zero_timeout_ms", 5000);
    const auto enable_timeout_ms = declare_parameter<int64_t>("enable_timeout_ms", 1000);
    const auto control_topic = declare_parameter<std::string>("control_topic", "/arm/can/control");
    const auto status_topic = declare_parameter<std::string>("status_topic", "/arm/can/status");
    const auto positions_topic = declare_parameter<std::string>("positions_topic", "/arm/can/positions");

    if (discovery_timeout_ms <= 0 || zero_timeout_ms <= 0 || enable_timeout_ms <= 0)
        throw std::invalid_argument("Arm startup timeouts must be positive");

    _discovery_timeout = std::chrono::milliseconds(discovery_timeout_ms);
    _zero_timeout = std::chrono::milliseconds(zero_timeout_ms);
    _enable_timeout = std::chrono::milliseconds(enable_timeout_ms);

    _can_interface.emplace(can_bus);
    _packet_manager.emplace(*_can_interface);
    _packet_manager->add_group(_control_board);  // Sends one startup probe per joint.

    _control_subscriber = create_subscription<actuator_msgs::msg::Actuators>(
        control_topic,
        10,
        std::bind(&ArmDriver::_handle_arm_control, this, std::placeholders::_1));
    _status_publisher =
        create_publisher<std_msgs::msg::Float64MultiArray>(status_topic, 10);
    _position_publisher =
        create_publisher<std_msgs::msg::Float64MultiArray>(positions_topic, 10);

    _packet_timer =
        create_wall_timer(PACKET_HANDLE_PERIOD, std::bind(&ArmDriver::_handle_can, this));
    _startup_timer =
        create_wall_timer(STARTUP_PERIOD, std::bind(&ArmDriver::_initialize_arm_board, this));
    _status_timer =
        create_wall_timer(STATUS_PERIOD, std::bind(&ArmDriver::_publish_status_messages, this));
    _position_timer =
        create_wall_timer(STATUS_PERIOD, std::bind(&ArmDriver::_publish_motor_positions, this));

    RCLCPP_INFO(
        get_logger(),
        "Arm CAN driver started on %s; probing %zu joints",
        can_bus.c_str(),
        JOINT_COUNT);
}

ArmDriver::~ArmDriver()
{
    cleanup();
}

void ArmDriver::_handle_can()
{
    try
    {
        if (_packet_manager)
            _packet_manager->handle();
    }
    catch (const std::exception& error)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000, "CAN handling failed: %s", error.what());
    }
}

void ArmDriver::_initialize_arm_board()
{
    switch (_startup_phase)
    {
    case startup_phase::DISCOVERING:
        if (std::chrono::steady_clock::now() - _phase_started >= _discovery_timeout)
            _finish_discovery();
        break;
    case startup_phase::ZEROING:
        _advance_zeroing();
        break;
    case startup_phase::ENABLING:
        _advance_enabling();
        break;
    case startup_phase::RUNNING:
        break;
    }
}

void ArmDriver::_finish_discovery()
{
    std::size_t ready_count = 0;

    for (const auto joint : ALL_JOINTS)
    {
        auto& startup = _joint_startup[_joint_index(joint)];
        const auto& status = _control_board.get_status(joint);

        if (!status.has_responded)
        {
            _report_failure(joint, "no response to probe");
            continue;
        }
        if (status.state.has_fault())
        {
            _report_failure(
                joint, "reported fault " + std::to_string(status.state.fault_code));
            continue;
        }
        if (!status.state.is_ready())
        {
            _report_failure(joint, "responded but is not ready");
            continue;
        }

        startup.candidate = true;
        ++ready_count;
        _transmit(parameter_group::make_zero_packet(joint));
    }

    _startup_phase = startup_phase::ZEROING;
    _phase_started = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "%zu joints ready; zeroing available joints", ready_count);

    if (ready_count == 0)
        _finish_startup();
}

void ArmDriver::_advance_zeroing()
{
    bool all_resolved = true;
    const bool timed_out =
        std::chrono::steady_clock::now() - _phase_started >= _zero_timeout;

    for (const auto joint : ALL_JOINTS)
    {
        auto& startup = _joint_startup[_joint_index(joint)];
        if (!startup.candidate)
            continue;

        const auto& status = _control_board.get_status(joint);
        if (status.state.has_fault())
        {
            startup.candidate = false;
            _report_failure(
                joint, "fault during zeroing: " + std::to_string(status.state.fault_code));
            continue;
        }

        if (status.state.is_zeroed())
        {
            if (!startup.enable_requested)
            {
                _transmit(parameter_group::make_enabled_packet(joint, true));
                startup.enable_requested = true;
            }
            continue;
        }

        if (timed_out)
        {
            startup.candidate = false;
            _report_failure(joint, "zeroing timed out");
        }
        else
        {
            all_resolved = false;
        }
    }

    if (all_resolved || timed_out)
    {
        _startup_phase = startup_phase::ENABLING;
        _phase_started = std::chrono::steady_clock::now();
    }
}

void ArmDriver::_advance_enabling()
{
    bool all_resolved = true;
    const bool timed_out =
        std::chrono::steady_clock::now() - _phase_started >= _enable_timeout;

    for (const auto joint : ALL_JOINTS)
    {
        auto& startup = _joint_startup[_joint_index(joint)];
        if (!startup.candidate)
            continue;

        const auto& status = _control_board.get_status(joint);
        if (status.state.has_fault())
        {
            startup.candidate = false;
            _report_failure(
                joint, "fault while enabling: " + std::to_string(status.state.fault_code));
        }
        else if (status.state.is_enabled())
        {
            startup.commandable = true;
        }
        else if (timed_out)
        {
            startup.candidate = false;
            _report_failure(joint, "enable timed out");
        }
        else
        {
            all_resolved = false;
        }
    }

    if (all_resolved || timed_out)
        _finish_startup();
}

void ArmDriver::_finish_startup()
{
    _startup_phase = startup_phase::RUNNING;
    if (_startup_timer)
    {
        _startup_timer->cancel();
        _startup_timer.reset();
    }

    const auto active_count = std::count_if(
        _joint_startup.begin(),
        _joint_startup.end(),
        [](const joint_startup_t& joint) { return joint.commandable; });
    RCLCPP_INFO(
        get_logger(),
        "Arm startup complete: %zu/%zu joints commandable",
        static_cast<std::size_t>(active_count),
        JOINT_COUNT);
}

void ArmDriver::_handle_arm_control(
    const actuator_msgs::msg::Actuators::SharedPtr message)
{
    if (_startup_phase != startup_phase::RUNNING)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000, "Ignoring arm command during startup");
        return;
    }

    if (message->position.empty())
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000, "Arm command contains no positions");
        return;
    }

    for (std::size_t index = 0; index < JOINT_COUNT && index < message->position.size();
         ++index)
    {
        const auto joint = ALL_JOINTS[index];
        auto& startup = _joint_startup[index];
        const auto& status = _control_board.get_status(joint);

        if (!startup.commandable)
            continue;
        if (status.state.has_fault())
        {
            startup.commandable = false;
            _report_failure(
                joint, "runtime fault " + std::to_string(status.state.fault_code));
            continue;
        }
        if (!status.state.is_ready() || !status.state.is_zeroed() ||
            !status.state.is_enabled())
        {
            startup.commandable = false;
            _report_failure(joint, "became unavailable");
            continue;
        }
        if (!std::isfinite(message->position[index]))
            continue;

        const double speed =
            index < message->velocity.size() && std::isfinite(message->velocity[index])
                ? message->velocity[index]
                : 0.0;
        const double acceleration =
            index < message->normalized.size() && std::isfinite(message->normalized[index])
                ? message->normalized[index]
                : 0.0;

        joint_command_t command{};
        command.position = _position_to_wire(message->position[index]);
        command.speed = _speed_to_wire(speed);
        command.acceleration = _acceleration_to_wire(acceleration);
        _transmit(parameter_group::make_command_packet(joint, command));
    }
}

void ArmDriver::_publish_status_messages()
{
    // Per-joint block: id, responded, ready, enabled, zeroed, fault,
    // position(rad), speed(rad/s), load, voltage(V), temperature(C),
    // current(A), moving.
    std_msgs::msg::Float64MultiArray message;
    message.data.reserve(JOINT_COUNT * 13);

    for (const auto joint : ALL_JOINTS)
    {
        const auto& status = _control_board.get_status(joint);
        auto& startup = _joint_startup[_joint_index(joint)];
        if (_startup_phase == startup_phase::RUNNING && startup.commandable &&
            (status.state.has_fault() || !status.state.is_ready() ||
             !status.state.is_zeroed() || !status.state.is_enabled()))
        {
            startup.commandable = false;
            _transmit(parameter_group::make_enabled_packet(joint, false));
            _report_failure(joint, "runtime state became unavailable");
        }

        message.data.emplace_back(static_cast<uint8_t>(joint));
        message.data.emplace_back(status.has_responded);
        message.data.emplace_back(status.state.is_ready());
        message.data.emplace_back(status.state.is_enabled());
        message.data.emplace_back(status.state.is_zeroed());
        message.data.emplace_back(status.state.fault_code);
        message.data.emplace_back(_position_from_wire(status.status_1.position));
        message.data.emplace_back(_speed_from_wire(status.status_1.speed));
        message.data.emplace_back(status.status_1.load);
        message.data.emplace_back(status.status_2.voltage / 1000.0);
        message.data.emplace_back(status.status_2.temperature);
        message.data.emplace_back(status.status_2.current / 1000.0);
        message.data.emplace_back(status.status_2.moving != 0);
    }

    _status_publisher->publish(message);
}

void ArmDriver::_publish_motor_positions()
{
    std_msgs::msg::Float64MultiArray message;
    message.data.reserve(JOINT_COUNT);

    for (const auto joint : ALL_JOINTS)
    {
        const auto& status = _control_board.get_status(joint);
        if (status.has_responded)
            message.data.emplace_back(_position_from_wire(status.status_1.position));
        else
            message.data.emplace_back(std::numeric_limits<double>::quiet_NaN());
    }

    _position_publisher->publish(message);
}

void ArmDriver::_report_failure(joint_id joint, std::string_view reason)
{
    auto& startup = _joint_startup[_joint_index(joint)];
    if (startup.failure_reported)
        return;

    startup.failure_reported = true;
    RCLCPP_ERROR(
        get_logger(),
        "Joint %s unavailable: %.*s; continuing without it",
        _joint_name(joint).data(),
        static_cast<int>(reason.size()),
        reason.data());
}

void ArmDriver::_transmit(const hi_can::Packet& packet)
{
    if (!_can_interface)
        return;

    try
    {
        _can_interface->transmit(packet);
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR(get_logger(), "CAN transmission failed: %s", error.what());
    }
}

std::size_t ArmDriver::_joint_index(joint_id joint)
{
    const auto iterator = std::find(ALL_JOINTS.begin(), ALL_JOINTS.end(), joint);
    if (iterator == ALL_JOINTS.end())
        throw std::invalid_argument("Unknown arm joint ID");
    return static_cast<std::size_t>(std::distance(ALL_JOINTS.begin(), iterator));
}

std::string_view ArmDriver::_joint_name(joint_id joint)
{
    switch (joint)
    {
    case joint_id::SHOULDER_PAN:
        return "shoulder_pan";
    case joint_id::SHOULDER_TILT:
        return "shoulder_tilt";
    case joint_id::ELBOW:
        return "elbow";
    case joint_id::WRIST_PITCH:
        return "wrist_pitch";
    case joint_id::WRIST_ROLL:
        return "wrist_roll";
    case joint_id::TOOL:
        return "tool";
    }
    return "unknown";
}

int16_t ArmDriver::_position_to_wire(double radians)
{
    const auto scaled = std::round(radians * MILLIRADIANS_PER_RADIAN);
    return static_cast<int16_t>(std::clamp(
        scaled,
        static_cast<double>(std::numeric_limits<int16_t>::min()),
        static_cast<double>(std::numeric_limits<int16_t>::max())));
}

uint16_t ArmDriver::_speed_to_wire(double radians_per_second)
{
    const auto scaled = std::round(std::abs(radians_per_second) * MILLIRADIANS_PER_RADIAN);
    return static_cast<uint16_t>(std::clamp(
        scaled,
        0.0,
        static_cast<double>(std::numeric_limits<uint16_t>::max())));
}

uint8_t ArmDriver::_acceleration_to_wire(double normalized)
{
    return static_cast<uint8_t>(std::round(std::clamp(normalized, 0.0, 1.0) * 255.0));
}

double ArmDriver::_position_from_wire(int16_t milliradians)
{
    return milliradians / MILLIRADIANS_PER_RADIAN;
}

double ArmDriver::_speed_from_wire(int16_t milliradians_per_second)
{
    return milliradians_per_second / MILLIRADIANS_PER_RADIAN;
}

void ArmDriver::cleanup()
{
    if (_cleaned_up)
        return;
    _cleaned_up = true;

    if (_packet_timer)
        _packet_timer->cancel();
    if (_startup_timer)
        _startup_timer->cancel();
    if (_status_timer)
        _status_timer->cancel();
    if (_position_timer)
        _position_timer->cancel();

    for (std::size_t index = 0; index < JOINT_COUNT; ++index)
    {
        if (_joint_startup[index].commandable)
            _transmit(parameter_group::make_enabled_packet(ALL_JOINTS[index], false));
    }

    _packet_manager.reset();
    _can_interface.reset();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<ArmDriver>();
        rclcpp::spin(node);
        node->cleanup();
    }
    catch (const std::exception& error)
    {
        RCLCPP_FATAL(rclcpp::get_logger("arm_driver"), "Arm driver failed: %s", error.what());
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}

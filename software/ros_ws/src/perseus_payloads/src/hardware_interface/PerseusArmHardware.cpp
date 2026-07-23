#include "perseus_payloads/hardware_interface/PerseusArmHardware.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{
bool has_interface(
    const std::vector<hardware_interface::InterfaceInfo>& interfaces,
    std::string_view name)
{
    return std::any_of(
        interfaces.begin(),
        interfaces.end(),
        [name](const hardware_interface::InterfaceInfo& interface)
        {
            return interface.name == name;
        });
}

std::string interface_name(std::string_view joint, std::string_view interface)
{
    return std::string(joint) + "/" + std::string(interface);
}
}  // namespace

namespace perseus_payloads
{

hardware_interface::CallbackReturn PerseusArmHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& parameters)
{
    if (hardware_interface::SystemInterface::on_init(parameters) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (!_validate_hardware_info())
        return hardware_interface::CallbackReturn::ERROR;

    try
    {
        _default_acceleration =
            std::stod(_hardware_parameter("default_acceleration", "0.5"));
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Invalid default_acceleration hardware parameter: %s",
            error.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (!std::isfinite(_default_acceleration))
    {
        RCLCPP_ERROR(get_logger(), "default_acceleration must be finite");
        return hardware_interface::CallbackReturn::ERROR;
    }
    _default_acceleration = std::clamp(_default_acceleration, 0.0, 1.0);

    return hardware_interface::CallbackReturn::SUCCESS;
}

bool PerseusArmHardware::_validate_hardware_info() const
{
    if (info_.joints.size() != JOINT_COUNT)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Expected %zu arm joints, received %zu",
            JOINT_COUNT,
            info_.joints.size());
        return false;
    }

    for (const auto expected_name : JOINT_ORDER)
    {
        const auto joint = std::find_if(
            info_.joints.begin(),
            info_.joints.end(),
            [expected_name](const hardware_interface::ComponentInfo& candidate)
            {
                return candidate.name == expected_name;
            });

        if (joint == info_.joints.end())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Missing required arm joint '%.*s'",
                static_cast<int>(expected_name.size()),
                expected_name.data());
            return false;
        }

        if (!has_interface(
                joint->command_interfaces,
                hardware_interface::HW_IF_POSITION) ||
            !has_interface(
                joint->command_interfaces,
                hardware_interface::HW_IF_VELOCITY) ||
            !has_interface(
                joint->state_interfaces,
                hardware_interface::HW_IF_POSITION) ||
            !has_interface(
                joint->state_interfaces,
                hardware_interface::HW_IF_VELOCITY))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Joint '%s' must expose position and velocity command/state interfaces",
                joint->name.c_str());
            return false;
        }
    }

    return true;
}

hardware_interface::CallbackReturn PerseusArmHardware::on_configure(
    const rclcpp_lifecycle::State&)
{
    if (!_cache_interface_handles())
        return hardware_interface::CallbackReturn::ERROR;

    for (std::size_t index = 0; index < JOINT_COUNT; ++index)
    {
        double initial_position = 0.0;
        if (!get_state(
                _position_state_interfaces[index],
                initial_position,
                false) ||
            !std::isfinite(initial_position))
        {
            initial_position = 0.0;
        }

        set_state(
            _position_state_interfaces[index],
            initial_position,
            false);
        set_state(_velocity_state_interfaces[index], 0.0, false);
        set_command(
            _position_command_interfaces[index],
            initial_position,
            false);
        set_command(_velocity_command_interfaces[index], 0.0, false);
    }

    const auto node = get_node();
    if (!node)
    {
        RCLCPP_ERROR(get_logger(), "Hardware component node is unavailable");
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto command_topic =
        _hardware_parameter("command_topic", "/arm/can/control");
    const auto positions_topic =
        _hardware_parameter("positions_topic", "/arm/can/positions");

    _command_publisher =
        node->create_publisher<actuator_msgs::msg::Actuators>(command_topic, 10);
    _position_subscriber =
        node->create_subscription<std_msgs::msg::Float64MultiArray>(
            positions_topic,
            10,
            std::bind(
                &PerseusArmHardware::_position_callback,
                this,
                std::placeholders::_1));

    {
        std::scoped_lock lock(_feedback_mutex);
        _feedback_received = false;
        _new_feedback = false;
    }
    _active = false;

    RCLCPP_INFO(get_logger(), "Perseus arm hardware configured");
    return hardware_interface::CallbackReturn::SUCCESS;
}

bool PerseusArmHardware::_cache_interface_handles()
{
    try
    {
        for (std::size_t index = 0; index < JOINT_COUNT; ++index)
        {
            const auto joint = JOINT_ORDER[index];
            _position_state_interfaces[index] = get_state_interface_handle(
                interface_name(joint, hardware_interface::HW_IF_POSITION));
            _velocity_state_interfaces[index] = get_state_interface_handle(
                interface_name(joint, hardware_interface::HW_IF_VELOCITY));
            _position_command_interfaces[index] = get_command_interface_handle(
                interface_name(joint, hardware_interface::HW_IF_POSITION));
            _velocity_command_interfaces[index] = get_command_interface_handle(
                interface_name(joint, hardware_interface::HW_IF_VELOCITY));
        }
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Failed to cache arm interfaces: %s",
            error.what());
        return false;
    }
    return true;
}

hardware_interface::CallbackReturn PerseusArmHardware::on_activate(
    const rclcpp_lifecycle::State&)
{
    for (std::size_t index = 0; index < JOINT_COUNT; ++index)
    {
        double position = 0.0;
        if (!get_state(_position_state_interfaces[index], position, false) ||
            !std::isfinite(position))
        {
            position = 0.0;
        }
        set_command(_position_command_interfaces[index], position, false);
        set_command(_velocity_command_interfaces[index], 0.0, false);
    }

    _active = true;
    RCLCPP_INFO(get_logger(), "Perseus arm hardware activated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PerseusArmHardware::on_deactivate(
    const rclcpp_lifecycle::State&)
{
    _active = false;
    RCLCPP_INFO(get_logger(), "Perseus arm hardware deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

void PerseusArmHardware::_position_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr message)
{
    if (message->data.size() != JOINT_COUNT)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Expected %zu arm positions, received %zu",
            JOINT_COUNT,
            message->data.size());
        return;
    }

    std::scoped_lock lock(_feedback_mutex);
    std::copy(
        message->data.begin(),
        message->data.end(),
        _latest_positions.begin());
    _feedback_received = true;
    _new_feedback = true;
}

hardware_interface::return_type PerseusArmHardware::read(
    const rclcpp::Time&,
    const rclcpp::Duration& period)
{
    std::array<double, JOINT_COUNT> positions{};
    bool has_new_feedback = false;
    {
        std::scoped_lock lock(_feedback_mutex);
        if (_feedback_received && _new_feedback)
        {
            positions = _latest_positions;
            _new_feedback = false;
            has_new_feedback = true;
        }
    }

    if (!has_new_feedback)
        return hardware_interface::return_type::OK;

    const double period_seconds = period.seconds();
    for (std::size_t index = 0; index < JOINT_COUNT; ++index)
    {
        if (!std::isfinite(positions[index]))
        {
            set_state(_velocity_state_interfaces[index], 0.0, false);
            continue;
        }

        double previous_position = positions[index];
        get_state(
            _position_state_interfaces[index],
            previous_position,
            false);

        const double velocity =
            period_seconds > 0.0
                ? (positions[index] - previous_position) / period_seconds
                : 0.0;
        set_state(
            _position_state_interfaces[index],
            positions[index],
            false);
        set_state(
            _velocity_state_interfaces[index],
            velocity,
            false);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type PerseusArmHardware::write(
    const rclcpp::Time& time,
    const rclcpp::Duration&)
{
    if (!_active || !_command_publisher)
        return hardware_interface::return_type::OK;

    actuator_msgs::msg::Actuators command;
    command.header.stamp = time;
    command.position.resize(JOINT_COUNT);
    command.velocity.resize(JOINT_COUNT);
    command.normalized.assign(JOINT_COUNT, _default_acceleration);

    for (std::size_t index = 0; index < JOINT_COUNT; ++index)
    {
        if (!get_command(
                _position_command_interfaces[index],
                command.position[index],
                false) ||
            !get_command(
                _velocity_command_interfaces[index],
                command.velocity[index],
                false))
        {
            RCLCPP_ERROR(get_logger(), "Failed to read arm command interfaces");
            return hardware_interface::return_type::ERROR;
        }

        if (!std::isfinite(command.position[index]))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Joint '%.*s' has a non-finite position command",
                static_cast<int>(JOINT_ORDER[index].size()),
                JOINT_ORDER[index].data());
            return hardware_interface::return_type::ERROR;
        }
        if (!std::isfinite(command.velocity[index]))
            command.velocity[index] = 0.0;
    }

    _command_publisher->publish(command);
    return hardware_interface::return_type::OK;
}

std::string PerseusArmHardware::_hardware_parameter(
    std::string_view name,
    std::string default_value) const
{
    const auto parameter = info_.hardware_parameters.find(std::string(name));
    return parameter == info_.hardware_parameters.end()
               ? std::move(default_value)
               : parameter->second;
}

}  // namespace perseus_payloads

PLUGINLIB_EXPORT_CLASS(
    perseus_payloads::PerseusArmHardware,
    hardware_interface::SystemInterface)

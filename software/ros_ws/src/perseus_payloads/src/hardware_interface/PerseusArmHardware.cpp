#include "perseus_payloads/hardware_interface/PerseusArmHardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

hardware_interface::CallbackReturn PerseusArmHardware::on_init(const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize storage
    _hw_states_position.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    _hw_states_velocity.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    _hw_commands.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    _hw_commands_velocity.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo& joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 2)
        {
            RCLCPP_WARN(rclcpp::get_logger("PerseusArmHardware"), "Joint '%s' has %zu command interfaces.", joint.name.c_str(), joint.command_interfaces.size());
        }

        bool has_position = false;
        for (const auto& si : joint.state_interfaces)
        {
            if (si.name == hardware_interface::HW_IF_POSITION)
                has_position = true;
        }

        if (!has_position)
        {
            RCLCPP_FATAL(rclcpp::get_logger("PerseusArmHardware"), "Joint '%s' missing 'position' state interface.", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PerseusArmHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
    // Reset values
    std::fill(_hw_states_position.begin(), _hw_states_position.end(), 0.0);
    std::fill(_hw_states_velocity.begin(), _hw_states_velocity.end(), 0.0);
    std::fill(_hw_commands.begin(), _hw_commands.end(), 0.0);
    std::fill(_hw_commands_velocity.begin(), _hw_commands_velocity.end(), 0.0);

    RCLCPP_INFO(rclcpp::get_logger("PerseusArmHardware"), "Perseus Arm Hardware Configured");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PerseusArmHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_hw_states_position[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_hw_states_velocity[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PerseusArmHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_hw_commands[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_hw_commands_velocity[i]));
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn PerseusArmHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    for (size_t i = 0; i < _hw_states_position.size(); ++i)
    {
        if (!std::isnan(_hw_states_position[i]))
        {
            _hw_commands[i] = _hw_states_position[i];
        }
        else
        {
            _hw_commands[i] = 0.0;
        }
    }

    // Create internal node
    _node = rclcpp::Node::make_shared("perseus_arm_hardware_node");

    // Create publisher
    _rsbl_publisher = _node->create_publisher<perseus_msgs::msg::ArmControl>("/arm/rsbl/control", 10);

    // Create subscriber
    _rsbl_status_subscriber = _node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/arm/rsbl/status", 10, std::bind(&PerseusArmHardware::_rsbl_status_callback, this, std::placeholders::_1));

    RCLCPP_INFO(rclcpp::get_logger("PerseusArmHardware"), "Perseus Arm Hardware Activated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PerseusArmHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("PerseusArmHardware"), "Perseus Arm Hardware Deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PerseusArmHardware::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // created internal node to handle subscriptions and publishers
    if (_node)
    {
        rclcpp::spin_some(_node);
    }

    if (_latest_rsbl_status.size() >= 14)  // At 7 bits per servo
    {
        double tilt_steps = _latest_rsbl_status[0];
        double pan_steps = _latest_rsbl_status[7];

        double tilt_rad = (tilt_steps - 2048.0) / (4096.0 / (2.0 * M_PI));
        double pan_rad = (pan_steps - 2048.0) / (4096.0 / (2.0 * M_PI));

        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            if (info_.joints[i].name == "shoulder_tilt")
                _hw_states_position[i] = tilt_rad;
            if (info_.joints[i].name == "shoulder_pan")
                _hw_states_position[i] = pan_rad;
        }
    }

    return hardware_interface::return_type::OK;
}

void PerseusArmHardware::_rsbl_status_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    _latest_rsbl_status = msg->data;
}

hardware_interface::return_type PerseusArmHardware::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    if (_rsbl_publisher)
    {
        perseus_msgs::msg::ArmControl msg;
        msg.position.resize(2);
        msg.velocity.resize(2);

        double pan_cmd = 0.0;
        double tilt_cmd = 0.0;
        double pan_vel_cmd = 0.0;
        double tilt_vel_cmd = 0.0;

        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            if (info_.joints[i].name == "shoulder_pan")
            {
                pan_cmd = _hw_commands[i];
                if (!std::isnan(_hw_commands_velocity[i]))
                    pan_vel_cmd = _hw_commands_velocity[i];
            }
            if (info_.joints[i].name == "shoulder_tilt")
            {
                tilt_cmd = _hw_commands[i];
                if (!std::isnan(_hw_commands_velocity[i]))
                    tilt_vel_cmd = _hw_commands_velocity[i];
            }
        }

        msg.position[0] = tilt_cmd;
        msg.position[1] = pan_cmd;

        msg.velocity[0] = tilt_vel_cmd;
        msg.velocity[1] = pan_vel_cmd;
        msg.acceleration = 0;

        _rsbl_publisher->publish(msg);
    }

    // RMD Driver Integration (Elbow and Wrist) - Placeholder
    // TODO: Implement communication with RMD driver for elbow, wrist_pitch, wrist_roll

    return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(PerseusArmHardware, hardware_interface::SystemInterface)

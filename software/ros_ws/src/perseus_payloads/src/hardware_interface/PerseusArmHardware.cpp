#include "perseus_payloads/hardware_interface/PerseusArmHardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hi_can_address.hpp"
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
    std::fill(_hw_states_position.begin(), _hw_states_position.end(), 0.0);
    std::fill(_hw_states_velocity.begin(), _hw_states_velocity.end(), 0.0);
    std::fill(_hw_commands.begin(), _hw_commands.end(), 0.0);
    std::fill(_hw_commands_velocity.begin(), _hw_commands_velocity.end(), 0.0);
    _last_motor_steps.clear();

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
    _rsbl_publisher = _node->create_publisher<actuator_msgs::msg::Actuators>("/arm/rsbl/control", 10);

    // Create subscriber
    _rsbl_status_subscriber = _node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/arm/rsbl/status", 10, std::bind(&PerseusArmHardware::_rsbl_status_callback, this, std::placeholders::_1));

    RCLCPP_INFO(rclcpp::get_logger("PerseusArmHardware"), "Perseus Arm Hardware Activated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PerseusArmHardware::on_deactivate(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(rclcpp::get_logger("PerseusArmHardware"), "Perseus Arm Hardware Deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// TODO: wrist pitch and roll need to be added, and fake joint.

hardware_interface::return_type PerseusArmHardware::read(const rclcpp::Time&, const rclcpp::Duration&)
{
    if (_node)
    {
        rclcpp::spin_some(_node);
    }

    if (!_latest_rsbl_status.empty())
    {
        _rsbl_ever_received = true;

        using namespace hi_can::addressing::post_landing::arm::control_board;
        const size_t STATUS_BLOCK_SIZE = 8;
        
        if (_latest_rsbl_status.size() % STATUS_BLOCK_SIZE == 0)
        {
            for (size_t offset = 0; offset < _latest_rsbl_status.size(); offset += STATUS_BLOCK_SIZE)
            {
                int servo_id = static_cast<int>(_latest_rsbl_status[offset]);
                double servo_pos_steps = _latest_rsbl_status[offset + 2];
                double servo_speed_steps = _latest_rsbl_status[offset + 3]; 
                
                constexpr double GEARBOX_RATIO = 25.0;
                constexpr double MOTOR_STEPS_TO_JOINT_RAD = (2.0 * M_PI) / (GEARBOX_RATIO * 4096.0);
                double pos_rad = servo_pos_steps * MOTOR_STEPS_TO_JOINT_RAD;
                double vel_rad = servo_speed_steps * MOTOR_STEPS_TO_JOINT_RAD; 

                std::string joint_name = "";
                if (servo_id == static_cast<int>(group::SHOULDER_TILT)) joint_name = "shoulder_tilt";
                else if (servo_id == static_cast<int>(group::SHOULDER_PAN)) joint_name = "shoulder_pan";
                else if (servo_id == static_cast<int>(group::ELBOW)) joint_name = "elbow";

                if (!joint_name.empty())
                {
                    for (size_t i = 0; i < info_.joints.size(); ++i)
                    {
                        if (info_.joints[i].name == joint_name)
                        {
                            _hw_states_position[i] = pos_rad;
                            _hw_states_velocity[i] = vel_rad;
                            break;
                        }
                    }
                }
            }
        }
    }

    // Mirror command as state for non-RSBL joints only (wrist, fake_dof)
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        if (info_.joints[i].name != "shoulder_tilt" && 
            info_.joints[i].name != "shoulder_pan" && 
            info_.joints[i].name != "elbow")
        {
            if (!std::isnan(_hw_commands[i]))
                _hw_states_position[i] = _hw_commands[i];
            _hw_states_velocity[i] = 0.0;
        }
    }

    return hardware_interface::return_type::OK;
}

void PerseusArmHardware::_rsbl_status_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    _latest_rsbl_status = msg->data;
}

hardware_interface::return_type PerseusArmHardware::write(const rclcpp::Time&, const rclcpp::Duration&)
{
    // TODO:
    // Change elbow to the big motors
    // add wrist pitch and roll
    // add fake_dof
    if (_rsbl_publisher)
    {
        actuator_msgs::msg::Actuators msg;
        msg.position.resize(3);
        msg.velocity.resize(3);
        msg.normalized.resize(3, 0.0);

        // Gearbox motor steps to joint radians
        constexpr double GEARBOX_RATIO = 25.0;
        constexpr double JOINT_RAD_TO_MOTOR_STEPS = GEARBOX_RATIO * 4096.0 / (2.0 * M_PI);

        const std::array<std::string, 3> joint_order = {"shoulder_tilt", "shoulder_pan", "elbow"};

        for (size_t j = 0; j < joint_order.size(); ++j)
        {
            const std::string& name = joint_order[j];
            double vel_rad = 0.0;
            double cmd_rad = 0.0;

            for (size_t i = 0; i < info_.joints.size(); ++i)
            {
                if (info_.joints[i].name == name)
                {
                    if (!std::isnan(_hw_commands[i]))
                        cmd_rad = _hw_commands[i];
                    if (!std::isnan(_hw_commands_velocity[i]))
                        vel_rad = _hw_commands_velocity[i];
                    break;
                }
            }

            // Compute target in motor steps (int32 to avoid overflow)
            int32_t target_steps = static_cast<int32_t>(cmd_rad * JOINT_RAD_TO_MOTOR_STEPS);

            // Get last known sent position. If first time, initialize from current hardware state!
            int32_t last = 0;
            if (_last_motor_steps.count(name))
            {
                last = _last_motor_steps[name];
            }
            else
            {
                // Find current state for this joint to initialize
                for (size_t i = 0; i < info_.joints.size(); ++i)
                {
                    if (info_.joints[i].name == name)
                    {
                        last = static_cast<int32_t>(_hw_states_position[i] * JOINT_RAD_TO_MOTOR_STEPS);
                        break;
                    }
                }
            }

            // Delta = how far to move from last sent position
            int32_t delta = target_steps - last;

            // Clamp to int16_t range to prevent CAN overflow
            if (delta > 32767) delta = 32767;
            if (delta < -32768) delta = -32768;

            _last_motor_steps[name] = last + delta;

            msg.position[j] = static_cast<double>(static_cast<int16_t>(delta));
            msg.velocity[j] = std::abs(vel_rad) * JOINT_RAD_TO_MOTOR_STEPS;
        }

        _rsbl_publisher->publish(msg);
    }

    return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(PerseusArmHardware, hardware_interface::SystemInterface)

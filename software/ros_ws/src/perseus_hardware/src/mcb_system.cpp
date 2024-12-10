#include "mcb_system.hpp"

#include <rclcpp/rclcpp.hpp>

using namespace perseus_hardware;

hardware_interface::CallbackReturn McbSystemHardware::on_init(const hardware_interface::HardwareInfo& info)
{
    if (const auto& ret = hardware_interface::SystemInterface::on_init(info); ret != hardware_interface::CallbackReturn::SUCCESS)
        return ret;

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> McbSystemHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // for (auto i = 0u; i < info_.joints.size(); i++)
    // {
    //     state_interfaces.emplace_back(hardware_interface::StateInterface(
    //         info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    //     state_interfaces.emplace_back(hardware_interface::StateInterface(
    //         info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    // }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> McbSystemHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    // for (auto i = 0u; i < info_.joints.size(); i++)
    // {
    //     command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //         info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    // }

    return command_interfaces;
}

hardware_interface::CallbackReturn McbSystemHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    // RCLCPP_INFO(get_logger(), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn McbSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    // RCLCPP_INFO(get_logger(), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type McbSystemHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type McbSystemHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    return hardware_interface::return_type::OK;
}

#include <pluginlib/pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(perseus_hardware::McbSystemHardware, hardware_interface::SystemInterface)

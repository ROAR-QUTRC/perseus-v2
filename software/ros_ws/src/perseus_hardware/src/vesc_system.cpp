#include "vesc_system.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hi_can_raw.hpp>
#include <rclcpp/rclcpp.hpp>

#include "system_common.hpp"

using namespace perseus_hardware;

hardware_interface::CallbackReturn VescSystemHardware::on_init(const hardware_interface::HardwareInfo& info)
{
    if (const auto& ret = hardware_interface::SystemInterface::on_init(info); ret != hardware_interface::CallbackReturn::SUCCESS)
        return ret;

    _logger = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("resource_manager.VescSystemHardware"));
    _clock = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

    CHECK_HARDWARE_PARAMETER_EXISTS(get_logger(), info, "can_bus");

    for (const auto& joint : info.joints)
    {
        CHECK_INTERFACE_COUNT(get_logger(), joint, command_interfaces, "command", 1);
        CHECK_INTERFACE_COUNT(get_logger(), joint, state_interfaces, "state", 2);

        CHECK_INTERFACE_NAME(get_logger(), joint, command_interfaces, "command", 0, hardware_interface::HW_IF_VELOCITY);
        CHECK_INTERFACE_NAME(get_logger(), joint, state_interfaces, "state", 0, hardware_interface::HW_IF_POSITION);
        CHECK_INTERFACE_NAME(get_logger(), joint, state_interfaces, "state", 1, hardware_interface::HW_IF_VELOCITY);

        CHECK_PARAMETER_EXISTS(get_logger(), joint, "id");
        // unsigned long vescId = std::stoul(joint.parameters.at("id"));

        _commandSpeeds.emplace_back(0);
        _realPositions.emplace_back(0);
        _realSpeeds.emplace_back(0);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VescSystemHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    const auto& info = get_info();
    for (size_t i = 0; i < info.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info.joints[i].name, hardware_interface::HW_IF_POSITION, &_realPositions[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_realSpeeds[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VescSystemHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    const auto& info = get_info();
    for (size_t i = 0; i < info.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_commandSpeeds[i]));
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn VescSystemHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    const auto& info = get_info();
    try
    {
        hi_can::RawCanInterface canInterface(info.hardware_parameters.at("can_bus"));
        _packetManager.emplace(canInterface);
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(get_logger(), "Failed to initialise CAN bus (%s): %s",
                     info.hardware_parameters.at("can_bus").c_str(), e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // RCLCPP_INFO(get_logger(), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    _packetManager.reset();

    // RCLCPP_INFO(get_logger(), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type VescSystemHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    _packetManager->handleReceive();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type VescSystemHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    _packetManager->handleTransmit();
    return hardware_interface::return_type::OK;
}

#include <pluginlib/pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(perseus_hardware::VescSystemHardware, hardware_interface::SystemInterface)

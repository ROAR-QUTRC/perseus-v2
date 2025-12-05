#include "vesc_system.hpp"

#include <cmath>
#include <cstdint>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hi_can_raw.hpp>
#include <limits>
#include <numbers>
#include <rclcpp/rclcpp.hpp>

#include "system_common.hpp"

#define GEARBOX_RATIO 40.0
#define ERPM_DIVISOR  7.0

using namespace perseus_hardware;
using namespace hardware_interface;

VescSystemHardware::~VescSystemHardware()
{
    // Workaround for ros2_control#472 because they STILL haven't fixed it after over 3 years!
    if (_packet_manager)
        on_deactivate(rclcpp_lifecycle::State());
    on_cleanup(rclcpp_lifecycle::State());
}

CallbackReturn VescSystemHardware::on_init(const HardwareInfo& info)
{
    if (const auto& ret = SystemInterface::on_init(info);
        ret != CallbackReturn::SUCCESS)
        return ret;

    CHECK_HARDWARE_PARAMETER_EXISTS(get_logger(), info, "can_bus");

    for (const auto& joint : info.joints)
    {
        CHECK_EQ_INTERFACE_COUNT(get_logger(), joint, command_interfaces, "command", 1);
        CHECK_EQ_INTERFACE_COUNT(get_logger(), joint, state_interfaces, "state", 4);

        CHECK_INTERFACE_NAME(get_logger(), joint, command_interfaces, "command", 0, HW_IF_VELOCITY);
        CHECK_INTERFACE_NAME(get_logger(), joint, state_interfaces, "state", 0, HW_IF_POSITION);
        CHECK_INTERFACE_NAME(get_logger(), joint, state_interfaces, "state", 1, HW_IF_VELOCITY);
        CHECK_INTERFACE_NAME(get_logger(), joint, state_interfaces, "state", 2, HW_IF_CURRENT);
        CHECK_INTERFACE_NAME(get_logger(), joint, state_interfaces, "state", 3, HW_IF_TEMPERATURE);

        CHECK_PARAMETER_EXISTS(get_logger(), joint, "id");
        try
        {
            unsigned long vesc_id = std::stoul(joint.parameters.at("id"));
            _vesc_ids.emplace_back(vesc_id);
        }
        catch (const std::exception& e)
        {
            RCLCPP_FATAL(get_logger(), "Failed to parse ID for joint '%s': %s", joint.name.c_str(), e.what());
            return CallbackReturn::ERROR;
        }

        _command_speeds.emplace_back(0);
        _real_positions.emplace_back(0);
        _real_speeds.emplace_back(0);
        _real_currents.emplace_back(0);
        // note: this is in degrees C
        _real_temperatures.emplace_back(25);
    }

    _last_transmit_error = get_clock()->now();

    return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> VescSystemHardware::export_state_interfaces()
{
    std::vector<StateInterface> state_interfaces;
    const auto& info = get_hardware_info();
    for (size_t i = 0; i < info.joints.size(); i++)
    {
        state_interfaces.emplace_back(StateInterface(
            info.joints[i].name, HW_IF_POSITION, &_real_positions[i]));
        state_interfaces.emplace_back(StateInterface(
            info.joints[i].name, HW_IF_VELOCITY, &_real_speeds[i]));
        state_interfaces.emplace_back(StateInterface(
            info.joints[i].name, HW_IF_CURRENT, &_real_currents[i]));
        state_interfaces.emplace_back(StateInterface(
            info.joints[i].name, HW_IF_TEMPERATURE, &_real_temperatures[i]));
    }

    return state_interfaces;
}

std::vector<CommandInterface> VescSystemHardware::export_command_interfaces()
{
    std::vector<CommandInterface> command_interfaces;
    const auto& info = get_hardware_info();
    for (size_t i = 0; i < info.joints.size(); i++)
    {
        command_interfaces.emplace_back(CommandInterface(
            info.joints[i].name, HW_IF_VELOCITY, &_command_speeds[i]));
    }

    return command_interfaces;
}

CallbackReturn VescSystemHardware::on_configure(
    const rclcpp_lifecycle::State&)
{
    const auto& info = get_hardware_info();

    try
    {
        _can_interface.emplace(info.hardware_parameters.at("can_bus"));
        _packet_manager.emplace(*_can_interface);
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(get_logger(), "Failed to initialise CAN bus '%s': %s",
                     info.hardware_parameters.at("can_bus").c_str(), e.what());
        _packet_manager.reset();
        return CallbackReturn::ERROR;
    }

    try
    {
        // We have to reserve the memory to prevent reallocations,
        // otherwise the references to the parameter groups will be invalidated
        _parameter_groups.reserve(_vesc_ids.size());
    }
    catch (std::exception& e)
    {
        RCLCPP_FATAL(get_logger(), "Failed to reserve memory for ESC parameter groups: %s", e.what());
        return CallbackReturn::ERROR;
    }

    for (const auto id : _vesc_ids)
    {
        try
        {
            using namespace hi_can;
            using namespace addressing::drive::vesc;

            _parameter_groups.emplace_back(id);
            _packet_manager->add_group(_parameter_groups.back());
        }
        catch (const std::exception& e)
        {
            RCLCPP_FATAL(get_logger(), "Failed to set up ESC parameter group %02lu: %s",
                         id, e.what());
        }
    }

    RCLCPP_INFO(get_logger(), "Successfully configured!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn VescSystemHardware::on_cleanup(
    const rclcpp_lifecycle::State&)
{
    _packet_manager.reset();
    _can_interface.reset();
    _parameter_groups.clear();

    for (auto& speed : _command_speeds)
        speed = 0;
    for (auto& speed : _real_speeds)
        speed = 0;
    for (auto& position : _real_positions)
        position = 0;
    for (auto& current : _real_currents)
        current = 0;
    for (auto& temperature : _real_temperatures)
        temperature = 25;

    RCLCPP_INFO(get_logger(), "Successfully cleaned up!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn VescSystemHardware::on_activate(
    const rclcpp_lifecycle::State&)
{
    // No enable line or similar applicable to VESCs
    RCLCPP_INFO(get_logger(), "Successfully activated!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn VescSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State&)
{
    for (auto& group : _parameter_groups)
        group.get_set_rpm().value = 0;

    try
    {
        _packet_manager->handle_transmit(true);
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN(get_logger(), "Error transmitting motor stop in deactivation: %s", e.what());
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(), "Successfully deactivated!");
    return CallbackReturn::SUCCESS;
}

return_type VescSystemHardware::read(
    const rclcpp::Time&, const rclcpp::Duration&)
{
    try
    {
        _packet_manager->handle_receive();
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Receive failed: %s", e.what());
    }

    for (size_t i = 0; i < _command_speeds.size(); i++)
    {
        auto& param_group = _parameter_groups[i];
        const auto status = param_group.get_status1();
        const auto motor_RPM = status.rpm / ERPM_DIVISOR;

        // note (and same for write() but in reverse):
        // We need to convert from motor RPM to wheel radians per second,
        // accounting for the ~1:40 gearbox in between.
        // This should probably be handled with a transmission,
        // but they're a pain in ros2_control right now.
        double wheel_RPM = motor_RPM / GEARBOX_RATIO;
        double revs_per_second = wheel_RPM / 60.0;
        double radians_per_second = revs_per_second * (2 * std::numbers::pi);

        _real_speeds[i] = radians_per_second;
        _real_currents[i] = status.current;

        const auto status4 = param_group.get_status4();
        // TODO: Test to see which would be more appropriate to use
        // (probably motor temp)
        _real_temperatures[i] = std::max(status4.temp_fet, status4.temp_motor);

        const auto status5 = param_group.get_status5();
        double wheel_tachometer = status5.tachometer / GEARBOX_RATIO;
        double wheel_revolutions = wheel_tachometer / ERPM_DIVISOR;
        _real_positions[i] = wheel_revolutions * (2 * std::numbers::pi);
    }

    return return_type::OK;
}

return_type VescSystemHardware::write(
    const rclcpp::Time&, const rclcpp::Duration&)
{
    for (size_t i = 0; i < _command_speeds.size(); i++)
    {
        // see comment in read() on the math
        const auto& radians_per_second = _command_speeds[i];
        double revs_per_second = radians_per_second / (2 * std::numbers::pi);
        double wheel_RPM = revs_per_second * 60.0;
        double motor_RPM = wheel_RPM * GEARBOX_RATIO;

        motor_RPM = std::clamp(motor_RPM * ERPM_DIVISOR, static_cast<double>(INT32_MIN), static_cast<double>(INT32_MAX));
        _parameter_groups[i].get_set_rpm().value = static_cast<int32_t>(std::round(motor_RPM));
    }

    try
    {
        if (get_clock()->now() - _last_transmit_error > std::chrono::milliseconds(1000))
            _packet_manager->handle_transmit();
    }
    catch (std::exception& e)
    {
        RCLCPP_WARN(get_logger(), "Transmit failed: %s", e.what());
        _last_transmit_error = get_clock()->now();
    }

    return return_type::OK;
}

#include <pluginlib/pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(perseus_hardware::VescSystemHardware, SystemInterface)

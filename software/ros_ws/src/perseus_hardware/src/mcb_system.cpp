#include "mcb_system.hpp"

#include <cmath>
#include <cstdint>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hi_can_raw.hpp>
#include <limits>
#include <numbers>
#include <rclcpp/rclcpp.hpp>

#include "system_common.hpp"

#define GEARBOX_RATIO 40.0

using namespace perseus_hardware;
using namespace hardware_interface;

McbSystemHardware::~McbSystemHardware()
{
    // Workaround for ros2_control#472 because they STILL haven't fixed it after over 3 years!
    if (_packetManager)
        on_deactivate(rclcpp_lifecycle::State());
    on_cleanup(rclcpp_lifecycle::State());
}

CallbackReturn McbSystemHardware::on_init(const HardwareInfo& info)
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
            unsigned long escId = std::stoul(joint.parameters.at("id"));
            _escIds.emplace_back(escId);
        }
        catch (const std::exception& e)
        {
            RCLCPP_FATAL(get_logger(), "Failed to parse ID for joint '%s': %s", joint.name.c_str(), e.what());
            return CallbackReturn::ERROR;
        }

        _commandSpeeds.emplace_back(0);
        _realPositions.emplace_back(0);
        _realSpeeds.emplace_back(0);
        _realCurrents.emplace_back(0);
        // note: this is in degrees C
        _realTemperatures.emplace_back(25);
    }

    _lastTransmitError = get_clock()->now();

    return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> McbSystemHardware::export_state_interfaces()
{
    std::vector<StateInterface> stateInterfaces;
    const auto& info = get_hardware_info();
    for (size_t i = 0; i < info.joints.size(); i++)
    {
        stateInterfaces.emplace_back(StateInterface(
            info.joints[i].name, HW_IF_POSITION, &_realPositions[i]));
        stateInterfaces.emplace_back(StateInterface(
            info.joints[i].name, HW_IF_VELOCITY, &_realSpeeds[i]));
        stateInterfaces.emplace_back(StateInterface(
            info.joints[i].name, HW_IF_CURRENT, &_realCurrents[i]));
        stateInterfaces.emplace_back(StateInterface(
            info.joints[i].name, HW_IF_TEMPERATURE, &_realTemperatures[i]));
    }

    return stateInterfaces;
}

std::vector<CommandInterface> McbSystemHardware::export_command_interfaces()
{
    std::vector<CommandInterface> commandInterfaces;
    const auto& info = get_hardware_info();
    for (size_t i = 0; i < info.joints.size(); i++)
    {
        commandInterfaces.emplace_back(CommandInterface(
            info.joints[i].name, HW_IF_VELOCITY, &_commandSpeeds[i]));
    }

    return commandInterfaces;
}

CallbackReturn McbSystemHardware::on_configure(
    const rclcpp_lifecycle::State&)
{
    const auto& info = get_hardware_info();

    try
    {
        _canInterface.emplace(info.hardware_parameters.at("can_bus"));
        _packetManager.emplace(*_canInterface);
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(get_logger(), "Failed to initialise CAN bus '%s': %s",
                     info.hardware_parameters.at("can_bus").c_str(), e.what());
        _packetManager.reset();
        return CallbackReturn::ERROR;
    }

    try
    {
        // We have to reserve the memory to prevent reallocations,
        // otherwise the references to the parameter groups will be invalidated
        _parameterGroups.reserve(_escIds.size());
    }
    catch (std::exception& e)
    {
        RCLCPP_FATAL(get_logger(), "Failed to reserve memory for ESC parameter groups: %s", e.what());
        return CallbackReturn::ERROR;
    }

    for (const auto id : _escIds)
    {
        try
        {
            using namespace hi_can;
            using namespace addressing::legacy;

            _parameterGroups.emplace_back(address_t(
                drive::SYSTEM_ID,
                drive::motors::SUBSYSTEM_ID,
                id));
            _packetManager->addGroup(_parameterGroups.back());
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

CallbackReturn McbSystemHardware::on_cleanup(
    const rclcpp_lifecycle::State&)
{
    _packetManager.reset();
    _canInterface.reset();
    _parameterGroups.clear();

    for (auto& speed : _commandSpeeds)
        speed = 0;
    for (auto& speed : _realSpeeds)
        speed = 0;
    for (auto& position : _realPositions)
        position = 0;
    for (auto& current : _realCurrents)
        current = 0;
    for (auto& temperature : _realTemperatures)
        temperature = 25;

    RCLCPP_INFO(get_logger(), "Successfully cleaned up!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn McbSystemHardware::on_activate(
    const rclcpp_lifecycle::State&)
{
    for (auto& group : _parameterGroups)
    {
        auto& speed = group.getSpeed();
        speed.enabled = true;
        speed.direction = hi_can::parameters::legacy::drive::motors::motor_direction::STOP;
        speed.speed = 0;
    }
    RCLCPP_INFO(get_logger(), "Successfully activated!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn McbSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State&)
{
    for (auto& group : _parameterGroups)
    {
        auto& speed = group.getSpeed();
        speed.enabled = false;
        speed.direction = hi_can::parameters::legacy::drive::motors::motor_direction::STOP;
        speed.speed = 0;
    }

    try
    {
        _packetManager->handleTransmit(true);
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN(get_logger(), "Error transmitting motor stop in deactivation: %s", e.what());
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(), "Successfully deactivated!");
    return CallbackReturn::SUCCESS;
}

return_type McbSystemHardware::read(
    const rclcpp::Time&, const rclcpp::Duration&)
{
    try
    {
        _packetManager->handleReceive();
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Receive failed: %s", e.what());
    }

    for (size_t i = 0; i < _commandSpeeds.size(); i++)
    {
        auto& paramGroup = _parameterGroups[i];
        const auto& status = paramGroup.getStatus();
        const auto& motorRPM = status.realSpeed;

        // note (and same for write() but in reverse):
        // We need to convert from motor RPM to wheel radians per second,
        // accounting for the ~1:40 gearbox in between.
        // This should probably be handled with a transmission,
        // but they're a pain in ros2_control right now.
        double wheelRPM = motorRPM / GEARBOX_RATIO;
        double revsPerSecond = wheelRPM / 60.0;
        double radiansPerSecond = revsPerSecond * (2 * std::numbers::pi);

        _realSpeeds[i] = radiansPerSecond;
        _realCurrents[i] = status.realCurrent / 1000.0;  // convert from mA to A
    }

    return return_type::OK;
}

return_type McbSystemHardware::write(
    const rclcpp::Time&, const rclcpp::Duration&)
{
    for (size_t i = 0; i < _commandSpeeds.size(); i++)
    {
        // see comment in read() on the math
        const auto& radiansPerSecond = _commandSpeeds[i];
        double revsPerSecond = radiansPerSecond / (2 * std::numbers::pi);
        double wheelRPM = revsPerSecond * 60.0;
        double motorRPM = wheelRPM * GEARBOX_RATIO;

        auto& speed = _parameterGroups[i].getSpeed();
        if (abs(motorRPM) < 0.1)
        {
            speed.direction = hi_can::parameters::legacy::drive::motors::motor_direction::STOP;
        }
        else if (motorRPM < 0)
            speed.direction = hi_can::parameters::legacy::drive::motors::motor_direction::REVERSE;
        else
            speed.direction = hi_can::parameters::legacy::drive::motors::motor_direction::FORWARD;

        motorRPM = std::abs(motorRPM);
        motorRPM = std::min(motorRPM, static_cast<double>(INT16_MAX));
        speed.speed = static_cast<int16_t>(std::round(motorRPM));
    }

    try
    {
        if (get_clock()->now() - _lastTransmitError > std::chrono::milliseconds(1000))
            _packetManager->handleTransmit();
    }
    catch (std::exception& e)
    {
        RCLCPP_WARN(get_logger(), "Transmit failed: %s", e.what());
        _lastTransmitError = get_clock()->now();
    }

    return return_type::OK;
}

#include <pluginlib/pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(perseus_hardware::McbSystemHardware, SystemInterface)

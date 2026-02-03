#include "kibisis_hardware/kibisis_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kibisis_hardware
{

    hardware_interface::CallbackReturn KibisisSystemHardware::on_init(
        const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialize storage for each joint
        const size_t num_joints = info_.joints.size();
        hw_positions_.resize(num_joints, 0.0);
        hw_velocities_.resize(num_joints, 0.0);
        hw_commands_.resize(num_joints, 0.0);
        prev_positions_.resize(num_joints, 0.0);
        joint_configs_.resize(num_joints);

        // Parse joint configuration from URDF
        for (size_t i = 0; i < num_joints; ++i)
        {
            const auto& joint = info_.joints[i];
            joint_configs_[i].name = joint.name;

            // Parse GPIO pin parameters if available
            if (joint.parameters.count("gpio_pin_a"))
            {
                joint_configs_[i].gpio_pin_a =
                    std::stoi(joint.parameters.at("gpio_pin_a"));
            }
            if (joint.parameters.count("gpio_pin_b"))
            {
                joint_configs_[i].gpio_pin_b =
                    std::stoi(joint.parameters.at("gpio_pin_b"));
            }
            if (joint.parameters.count("encoder_pin_a"))
            {
                joint_configs_[i].encoder_pin_a =
                    std::stoi(joint.parameters.at("encoder_pin_a"));
            }
            if (joint.parameters.count("encoder_pin_b"))
            {
                joint_configs_[i].encoder_pin_b =
                    std::stoi(joint.parameters.at("encoder_pin_b"));
            }

            RCLCPP_INFO(
                rclcpp::get_logger("KibisisSystemHardware"),
                "Joint '%s': GPIO pins [%d, %d], Encoder pins [%d, %d]",
                joint_configs_[i].name.c_str(),
                joint_configs_[i].gpio_pin_a,
                joint_configs_[i].gpio_pin_b,
                joint_configs_[i].encoder_pin_a,
                joint_configs_[i].encoder_pin_b);

            // Verify command interfaces
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("KibisisSystemHardware"),
                    "Joint '%s' has %zu command interfaces. Expected 1 (velocity).",
                    joint.name.c_str(), joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name !=
                hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("KibisisSystemHardware"),
                    "Joint '%s' has command interface '%s'. Expected '%s'.",
                    joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(),
                    hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Verify state interfaces
            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("KibisisSystemHardware"),
                    "Joint '%s' has %zu state interfaces. Expected 2 (position, "
                    "velocity).",
                    joint.name.c_str(), joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        RCLCPP_INFO(
            rclcpp::get_logger("KibisisSystemHardware"),
            "Kibisis hardware interface initialized with %zu joints", num_joints);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    KibisisSystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            state_interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_positions_[i]);

            state_interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &hw_velocities_[i]);
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    KibisisSystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            command_interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &hw_commands_[i]);
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn KibisisSystemHardware::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("KibisisSystemHardware"),
            "Configuring Kibisis hardware interface...");

        // Reset state
        for (size_t i = 0; i < hw_positions_.size(); ++i)
        {
            hw_positions_[i] = 0.0;
            hw_velocities_[i] = 0.0;
            hw_commands_[i] = 0.0;
            prev_positions_[i] = 0.0;
        }

        // Initialize GPIO
        if (!init_gpio())
        {
            RCLCPP_WARN(
                rclcpp::get_logger("KibisisSystemHardware"),
                "GPIO initialization failed - running in simulation mode");
        }

        RCLCPP_INFO(
            rclcpp::get_logger("KibisisSystemHardware"),
            "Kibisis hardware interface configured");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KibisisSystemHardware::on_cleanup(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("KibisisSystemHardware"),
            "Cleaning up Kibisis hardware interface...");

        cleanup_gpio();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KibisisSystemHardware::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("KibisisSystemHardware"),
            "Activating Kibisis hardware interface...");

        // Reset commands to zero
        for (auto& cmd : hw_commands_)
        {
            cmd = 0.0;
        }

        RCLCPP_INFO(
            rclcpp::get_logger("KibisisSystemHardware"),
            "Kibisis hardware interface activated");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn KibisisSystemHardware::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("KibisisSystemHardware"),
            "Deactivating Kibisis hardware interface...");

        // Stop all motors
        for (size_t i = 0; i < hw_commands_.size(); ++i)
        {
            hw_commands_[i] = 0.0;
            set_motor_velocity(i, 0.0);
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type KibisisSystemHardware::read(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
    {
        const double dt = period.seconds();

        for (size_t i = 0; i < hw_positions_.size(); ++i)
        {
            // Read encoder position
            const double new_position = read_encoder(i);

            // Calculate velocity from position change
            if (dt > 0.0)
            {
                hw_velocities_[i] = (new_position - prev_positions_[i]) / dt;
            }

            hw_positions_[i] = new_position;
            prev_positions_[i] = new_position;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type KibisisSystemHardware::write(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        for (size_t i = 0; i < hw_commands_.size(); ++i)
        {
            set_motor_velocity(i, hw_commands_[i]);
        }

        return hardware_interface::return_type::OK;
    }

    bool KibisisSystemHardware::init_gpio()
    {
        // TODO: Implement GPIO initialization for Raspberry Pi
        // This will depend on the specific GPIO library being used
        // (e.g., pigpio, wiringPi, or direct /sys/class/gpio access)

        RCLCPP_INFO(
            rclcpp::get_logger("KibisisSystemHardware"),
            "GPIO initialization - placeholder implementation");

        // For now, return true to allow testing without actual hardware
        return true;
    }

    void KibisisSystemHardware::cleanup_gpio()
    {
        // TODO: Implement GPIO cleanup
        RCLCPP_INFO(
            rclcpp::get_logger("KibisisSystemHardware"),
            "GPIO cleanup - placeholder implementation");
    }

    void KibisisSystemHardware::set_motor_velocity(
        size_t joint_index, double velocity)
    {
        if (joint_index >= joint_configs_.size())
        {
            return;
        }

        // TODO: Implement actual motor control via GPIO/motor controller board
        // This will send PWM signals to the motor control board
        // Velocity is in rad/s, convert to appropriate motor command

        // For simulation/mock mode, we can directly mirror the command
        // In a real implementation, this would:
        // 1. Convert rad/s to motor PWM duty cycle
        // 2. Set direction based on sign of velocity
        // 3. Send PWM signal via GPIO

        (void)velocity;  // Suppress unused warning in placeholder
    }

    double KibisisSystemHardware::read_encoder(size_t joint_index)
    {
        if (joint_index >= joint_configs_.size())
        {
            return 0.0;
        }

        // TODO: Implement actual encoder reading
        // This will read quadrature encoder signals from GPIO pins

        // For simulation/mock mode, integrate commanded velocity
        // In a real implementation, this would:
        // 1. Read encoder pulse count
        // 2. Convert to radians based on encoder resolution

        // Mock implementation: return current position (no change in simulation)
        return hw_positions_[joint_index];
    }

}  // namespace kibisis_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    kibisis_hardware::KibisisSystemHardware,
    hardware_interface::SystemInterface)

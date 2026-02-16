#include "kibisis_hardware/kibisis_system.hpp"

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>

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
        const hardware_interface::HardwareComponentInterfaceParams& params)
    {
        if (hardware_interface::SystemInterface::on_init(params) !=
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

            RCLCPP_INFO(
                rclcpp::get_logger("KibisisSystemHardware"),
                "Joint '%s'",
                joint_configs_[i].name.c_str());

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
        if (!init_i2c())
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

        cleanup_i2c();

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

    bool KibisisSystemHardware::init_i2c()
    {
        RCLCPP_INFO(
            rclcpp::get_logger("KibisisSystemHardware"),
            "I2c initialised");

        _i2c_file = open(_i2c_filename.c_str(), O_RDWR, O_APPEND);
        if (_i2c_file < 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to open i2c path");
            return false;
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "Opened i2c bus at device: %s", _i2c_filename.c_str());
            return true;
        }
    }

    void KibisisSystemHardware::cleanup_i2c()
    {
        if (_i2c_file >= 0)
        {
            if (close(_i2c_file) < 0)
            {
                RCLCPP_FATAL(this->get_logger(), "Failed to close i2c bus at device: %s", _i2c_filename.c_str());
            }
        }
        RCLCPP_INFO(
            rclcpp::get_logger("KibisisSystemHardware"),
            "I2c cleanup");
    }

    void KibisisSystemHardware::set_motor_velocity(
        size_t joint_index, double velocity)
    {
        if (joint_index >= joint_configs_.size())
        {
            return;
        }
        // 0x00 = left forward
        // 0x01 = left backward
        // 0x02 = right forward
        // 0x03 = right backward
        uint8_t function = (joint_index * 2 + (velocity < 0));
        // TODO: Calculate velocity PWM value
        // Convert rad/s to motor PWM duty cycle
        uint16_t velocity_pwm = static_cast<uint16_t>(velocity);
        std::vector<uint8_t> send_buffer;
        send_buffer.push_back(function);
        send_buffer.push_back((velocity_pwm && 0xFF00) >> 8);
        send_buffer.push_back(velocity_pwm && 0xFF);
        i2c_msg message[] = {{
            .addr = _PICO_ADDRESS,
            .flags = 0,
            .len = static_cast<uint16_t>(send_buffer.size()),
            .buf = send_buffer.data(),
        }};
        i2c_rdwr_ioctl_data send_messages = {
            .msgs = message,
            .nmsgs = sizeof(message) / sizeof(message[0]),
        };
        if (ioctl(_i2c_file, I2C_RDWR, send_messages) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error sending motor velocity data! Code: %x PWM Value: %x", function, velocity_pwm);
        }
    }

    double KibisisSystemHardware::read_encoder(size_t joint_index)
    {
        if (joint_index >= joint_configs_.size())
        {
            return 0.0;
        }
        uint8_t* received_data = static_cast<uint8_t*>(malloc(2 * sizeof(uint8_t)));
        i2c_msg messages[] = {{
                                  .addr = _PICO_ADDRESS,
                                  .flags = 0,
                                  .len = 1,
                                  .buf = &(joint_index ? _READ_ENCODER_LEFT : _READ_ENCODER_RIGHT),
                              },
                              {
                                  .addr = _PICO_ADDRESS,
                                  .flags = I2C_M_RD,
                                  .len = 2,
                                  .buf = received_data,
                              }};
        i2c_rdwr_ioctl_data send_messages = {
            .msgs = messages,
            .nmsgs = sizeof(messages) / sizeof(messages[0]),
        };
        if (ioctl(_i2c_file, I2C_RDWR, send_messages))
        {
            RCLCPP_ERROR(this->get_logger(), "Error reading encoder data! Joint index: %ld", joint_index);
        }
        // TODO: Implement actual encoder calculations

        // For simulation/mock mode, integrate commanded velocity
        // In a real implementation, this would:
        // 1. Read encoder pulse count
        // 2. Convert to radians based on encoder resolution
        uint16_t encoder_reading = received_data[0] << 8 | received_data[1];
        free(received_data);
        // Mock implementation: return current position (no change in simulation)
        return hw_positions_[joint_index];
    }

}  // namespace kibisis_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    kibisis_hardware::KibisisSystemHardware,
    hardware_interface::SystemInterface)

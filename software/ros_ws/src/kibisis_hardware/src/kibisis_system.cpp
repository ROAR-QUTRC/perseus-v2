#include "kibisis_hardware/kibisis_system.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kibisis_hardware
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("KibisisSystemHardware");

// ---------------------------------------------------------------------------
// I2C helpers
// ---------------------------------------------------------------------------

bool KibisisSystemHardware::i2c_open()
{
    i2c_fd_ = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd_ < 0)
    {
        RCLCPP_ERROR(LOGGER, "Failed to open I2C device %s: %s",
            I2C_DEVICE, strerror(errno));
        return false;
    }
    if (ioctl(i2c_fd_, I2C_SLAVE, PICO_I2C_ADDRESS) < 0)
    {
        RCLCPP_ERROR(LOGGER, "Failed to set I2C slave address 0x%02X: %s",
            PICO_I2C_ADDRESS, strerror(errno));
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }
    RCLCPP_INFO(LOGGER, "Opened %s, Pico address 0x%02X", I2C_DEVICE, PICO_I2C_ADDRESS);
    return true;
}

void KibisisSystemHardware::i2c_close()
{
    if (i2c_fd_ >= 0)
    {
        close(i2c_fd_);
        i2c_fd_ = -1;
    }
}

bool KibisisSystemHardware::i2c_write_reg(uint8_t reg, const uint8_t* data, size_t len)
{
    if (i2c_fd_ < 0) return false;

    std::vector<uint8_t> buf(1 + len);
    buf[0] = reg;
    std::memcpy(buf.data() + 1, data, len);

    i2c_msg msg = {
        .addr  = PICO_I2C_ADDRESS,
        .flags = 0,
        .len   = static_cast<uint16_t>(buf.size()),
        .buf   = buf.data(),
    };
    i2c_rdwr_ioctl_data txn = { .msgs = &msg, .nmsgs = 1 };

    if (ioctl(i2c_fd_, I2C_RDWR, &txn) < 0)
    {
        RCLCPP_ERROR(LOGGER, "I2C write to reg 0x%02X failed: %s",
            reg, strerror(errno));
        return false;
    }
    return true;
}

bool KibisisSystemHardware::i2c_read_reg(uint8_t reg, uint8_t* data, size_t len)
{
    if (i2c_fd_ < 0) return false;

    i2c_msg msgs[2] = {
        {
            .addr  = PICO_I2C_ADDRESS,
            .flags = 0,
            .len   = 1,
            .buf   = &reg,
        },
        {
            .addr  = PICO_I2C_ADDRESS,
            .flags = I2C_M_RD,
            .len   = static_cast<uint16_t>(len),
            .buf   = data,
        },
    };
    i2c_rdwr_ioctl_data txn = { .msgs = msgs, .nmsgs = 2 };

    if (ioctl(i2c_fd_, I2C_RDWR, &txn) < 0)
    {
        RCLCPP_ERROR(LOGGER, "I2C read from reg 0x%02X failed: %s",
            reg, strerror(errno));
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn KibisisSystemHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params)
{
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const size_t num_joints = info_.joints.size();
    hw_positions_.assign(num_joints, 0.0);
    hw_velocities_.assign(num_joints, 0.0);
    hw_commands_.assign(num_joints, 0.0);
    prev_positions_.assign(num_joints, 0.0);

    for (size_t i = 0; i < num_joints; ++i)
    {
        const auto& joint = info_.joints[i];

        if (joint.command_interfaces.size() != 1 ||
            joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_ERROR(LOGGER,
                "Joint '%s': expected exactly 1 velocity command interface",
                joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces.size() != 2)
        {
            RCLCPP_ERROR(LOGGER,
                "Joint '%s': expected 2 state interfaces (position, velocity)",
                joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    RCLCPP_INFO(LOGGER, "Initialised with %zu drive joints", num_joints);
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

    // GPIO — moisture sensor
    state_interfaces.emplace_back("moisture_sensor", "adc_raw", &moisture_adc_state_);

    // GPIO — LDR sensors
    state_interfaces.emplace_back("ldr_sensor", "a_ambient",     &ldr_a_ambient_state_);
    state_interfaces.emplace_back("ldr_sensor", "b_ambient",     &ldr_b_ambient_state_);
    state_interfaces.emplace_back("ldr_sensor", "a_illuminated", &ldr_a_illuminated_state_);
    state_interfaces.emplace_back("ldr_sensor", "b_illuminated", &ldr_b_illuminated_state_);

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

    // GPIO — space motor speed (-100..100)
    command_interfaces.emplace_back("space_motor", "velocity", &space_motor_cmd_);

    // GPIO — moisture sensor trigger
    command_interfaces.emplace_back("moisture_sensor", "trigger", &moisture_trigger_cmd_);

    // GPIO — LDR trigger
    command_interfaces.emplace_back("ldr_sensor", "trigger", &ldr_trigger_cmd_);

    return command_interfaces;
}

hardware_interface::CallbackReturn KibisisSystemHardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    for (size_t i = 0; i < hw_positions_.size(); ++i)
    {
        hw_positions_[i]   = 0.0;
        hw_velocities_[i]  = 0.0;
        hw_commands_[i]    = 0.0;
        prev_positions_[i] = 0.0;
    }
    space_motor_cmd_          = 0.0;
    moisture_trigger_cmd_     = 0.0;
    moisture_adc_state_       = 0.0;
    ldr_trigger_cmd_          = 0.0;
    ldr_a_ambient_state_      = 0.0;
    ldr_b_ambient_state_      = 0.0;
    ldr_a_illuminated_state_  = 0.0;
    ldr_b_illuminated_state_  = 0.0;

    if (!i2c_open())
    {
        RCLCPP_WARN(LOGGER, "I2C open failed — hardware will not respond");
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KibisisSystemHardware::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    i2c_close();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KibisisSystemHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    for (auto& cmd : hw_commands_) cmd = 0.0;
    space_motor_cmd_          = 0.0;
    moisture_trigger_cmd_     = 0.0;
    ldr_trigger_cmd_          = 0.0;

    RCLCPP_INFO(LOGGER, "Activated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KibisisSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    const uint8_t zero = 0;
    i2c_write_reg(REG_MOTOR_A_SPEED,     &zero, 1);
    i2c_write_reg(REG_MOTOR_B_SPEED,     &zero, 1);
    i2c_write_reg(REG_SPACE_MOTOR_SPEED, &zero, 1);

    RCLCPP_INFO(LOGGER, "Deactivated — all motors stopped");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Read / Write
// ---------------------------------------------------------------------------

void KibisisSystemHardware::read_wheel_encoder(size_t joint_index)
{
    const uint8_t reg = (joint_index == 0) ? REG_ENC_A_COUNT_0 : REG_ENC_B_COUNT_0;

    uint8_t buf[4] = {};
    if (!i2c_read_reg(reg, buf, 4)) return;

    const int32_t counts = static_cast<int32_t>(
        static_cast<uint32_t>(buf[0])         |
        (static_cast<uint32_t>(buf[1]) << 8)  |
        (static_cast<uint32_t>(buf[2]) << 16) |
        (static_cast<uint32_t>(buf[3]) << 24));

    hw_positions_[joint_index] = static_cast<double>(counts) * RADS_PER_COUNT;
}

void KibisisSystemHardware::write_drive_motor(uint8_t reg, double rad_s)
{
    const double clamped = std::clamp(rad_s, -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S);
    const int8_t speed   = static_cast<int8_t>(
        std::round(clamped / MAX_WHEEL_RAD_S * 100.0));
    i2c_write_reg(reg, reinterpret_cast<const uint8_t*>(&speed), 1);
}

hardware_interface::return_type KibisisSystemHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    const double dt = period.seconds();

    // Wheel encoders
    for (size_t i = 0; i < hw_positions_.size(); ++i)
    {
        read_wheel_encoder(i);
        if (dt > 0.0)
            hw_velocities_[i] = (hw_positions_[i] - prev_positions_[i]) / dt;
        prev_positions_[i] = hw_positions_[i];
    }

    // Moisture sensor — always read, stable until next trigger
    {
        uint8_t buf[2] = {};
        if (i2c_read_reg(REG_MOISTURE_VALUE_0, buf, 2))
        {
            moisture_adc_state_ = static_cast<double>(
                static_cast<uint16_t>(buf[0]) |
                (static_cast<uint16_t>(buf[1]) << 8));
        }
    }

    // LDR sensors — always read, stable until next trigger
    {
        uint8_t buf[2] = {};

        if (i2c_read_reg(REG_LDR_A_AMBIENT_0, buf, 2))
            ldr_a_ambient_state_ = static_cast<double>(
                static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8));

        if (i2c_read_reg(REG_LDR_B_AMBIENT_0, buf, 2))
            ldr_b_ambient_state_ = static_cast<double>(
                static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8));

        if (i2c_read_reg(REG_LDR_A_ILLUMINATED_0, buf, 2))
            ldr_a_illuminated_state_ = static_cast<double>(
                static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8));

        if (i2c_read_reg(REG_LDR_B_ILLUMINATED_0, buf, 2))
            ldr_b_illuminated_state_ = static_cast<double>(
                static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8));
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type KibisisSystemHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // Drive motors
    write_drive_motor(REG_MOTOR_A_SPEED, hw_commands_[0]);
    write_drive_motor(REG_MOTOR_B_SPEED, hw_commands_[1]);

    // Space motor
    {
        const double clamped = std::clamp(space_motor_cmd_, -100.0, 100.0);
        const int8_t speed   = static_cast<int8_t>(std::round(clamped));
        i2c_write_reg(REG_SPACE_MOTOR_SPEED,
            reinterpret_cast<const uint8_t*>(&speed), 1);
    }

    // Moisture trigger — fires once and clears
    if (moisture_trigger_cmd_ >= 1.0)
    {
        const uint8_t trigger = 0x01;
        i2c_write_reg(REG_MOISTURE_SAMPLE, &trigger, 1);
        moisture_trigger_cmd_ = 0.0;
    }

    // LDR trigger — fires once and clears
    if (ldr_trigger_cmd_ >= 1.0)
    {
        const uint8_t trigger = 0x01;
        i2c_write_reg(REG_LDR_SAMPLE, &trigger, 1);
        ldr_trigger_cmd_ = 0.0;
    }

    return hardware_interface::return_type::OK;
}

}  // namespace kibisis_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    kibisis_hardware::KibisisSystemHardware,
    hardware_interface::SystemInterface)
#pragma once

#include <hardware_interface/system_interface.hpp>
#include <numbers>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <string>
#include <vector>

namespace kibisis_hardware
{

    /**
     * @brief Hardware interface for Kibisis robot
     *
     * Owns /dev/i2c-1 and communicates with the Kibisis Pico Driver (address 0x41).
     *
     * Joint interfaces (ros2_control standard):
     *   left_wheel_joint  - velocity command, position + velocity state (encoder)
     *   right_wheel_joint - velocity command, position + velocity state (encoder)
     *
     * GPIO interfaces (non-joint hardware exposed through ros2_control):
     *   space_motor/velocity        - command: speed -100..100
     *   moisture_sensor/trigger     - command: write 1.0 to request a sample
     *   moisture_sensor/adc_raw     - state:   raw 12-bit ADC value (0-4095)
     *   ldr_sensor/trigger          - command: write 1.0 to request a sample
     *   ldr_sensor/a_ambient        - state:   LDR A ambient reading (0-4095)
     *   ldr_sensor/b_ambient        - state:   LDR B ambient reading (0-4095)
     *   ldr_sensor/a_illuminated    - state:   LDR A illuminated reading (0-4095)
     *   ldr_sensor/b_illuminated    - state:   LDR B illuminated reading (0-4095)
     *
     * Pico register map:
     *   0x00  MOTOR_A_SPEED           write  int8_t   left wheel  -100..100
     *   0x01  MOTOR_B_SPEED           write  int8_t   right wheel -100..100
     *   0x02  SPACE_MOTOR_SPEED       write  int8_t   -100..100
     *   0x10  ENC_A_COUNT_0..3        read   int32_t  left encoder (little-endian)
     *   0x14  ENC_B_COUNT_0..3        read   int32_t  right encoder (little-endian)
     *   0x20  STATUS                  read   uint8_t  heartbeat
     *   0x30  MOISTURE_SAMPLE         write  uint8_t  write 1 to trigger sample
     *   0x31  MOISTURE_VALUE_0..1     read   uint16_t ADC result (little-endian)
     *   0x40  LDR_SAMPLE              write  uint8_t  write 1 to trigger sample
     *   0x41  LDR_A_AMBIENT_0..1      read   uint16_t LDR A ambient (little-endian)
     *   0x43  LDR_B_AMBIENT_0..1      read   uint16_t LDR B ambient (little-endian)
     *   0x45  LDR_A_ILLUMINATED_0..1  read   uint16_t LDR A illuminated (little-endian)
     *   0x47  LDR_B_ILLUMINATED_0..1  read   uint16_t LDR B illuminated (little-endian)
     */
    class KibisisSystemHardware final : public hardware_interface::SystemInterface
    {
    public:
        KibisisSystemHardware() = default;
        ~KibisisSystemHardware() override = default;

        KibisisSystemHardware(const KibisisSystemHardware&) = delete;
        KibisisSystemHardware& operator=(const KibisisSystemHardware&) = delete;
        KibisisSystemHardware(KibisisSystemHardware&&) = delete;
        KibisisSystemHardware& operator=(KibisisSystemHardware&&) = delete;

        [[nodiscard]] hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams& params) override;

        [[nodiscard]] std::vector<hardware_interface::StateInterface>
        export_state_interfaces() override;

        [[nodiscard]] std::vector<hardware_interface::CommandInterface>
        export_command_interfaces() override;

        [[nodiscard]] hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& previous_state) override;

        [[nodiscard]] hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State& previous_state) override;

        [[nodiscard]] hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state) override;

        [[nodiscard]] hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& previous_state) override;

        [[nodiscard]] hardware_interface::return_type read(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;

        [[nodiscard]] hardware_interface::return_type write(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        // Robot constants
        static constexpr int COUNTS_PER_REV = 6400;
        static constexpr double RADS_PER_COUNT =
            (2.0 * std::numbers::pi) / static_cast<double>(COUNTS_PER_REV);
        static constexpr double MAX_WHEEL_RAD_S = 10.47;

        // Pico I2C register addresses
        static constexpr uint8_t REG_MOTOR_A_SPEED = 0x00;
        static constexpr uint8_t REG_MOTOR_B_SPEED = 0x01;
        static constexpr uint8_t REG_SPACE_MOTOR_SPEED = 0x02;
        static constexpr uint8_t REG_ENC_A_COUNT_0 = 0x10;
        static constexpr uint8_t REG_ENC_B_COUNT_0 = 0x14;
        static constexpr uint8_t REG_MOISTURE_SAMPLE = 0x30;
        static constexpr uint8_t REG_MOISTURE_VALUE_0 = 0x31;
        static constexpr uint8_t REG_LDR_SAMPLE = 0x40;
        static constexpr uint8_t REG_LDR_A_AMBIENT_0 = 0x41;
        static constexpr uint8_t REG_LDR_B_AMBIENT_0 = 0x43;
        static constexpr uint8_t REG_LDR_A_ILLUMINATED_0 = 0x45;
        static constexpr uint8_t REG_LDR_B_ILLUMINATED_0 = 0x47;

        // I2C
        static constexpr uint8_t PICO_I2C_ADDRESS = 0x41;
        static constexpr const char* I2C_DEVICE = "/dev/i2c-1";
        int i2c_fd_ = -1;

        bool i2c_open();
        void i2c_close();
        bool i2c_write_reg(uint8_t reg, const uint8_t* data, size_t len);
        bool i2c_read_reg(uint8_t reg, uint8_t* data, size_t len);

        // Drive wheel joints
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
        std::vector<double> hw_commands_;
        std::vector<double> prev_positions_;

        void write_drive_motor(uint8_t reg, double rad_s);
        void read_wheel_encoder(size_t joint_index);

        // GPIO — space motor
        double space_motor_cmd_ = 0.0;

        // GPIO — moisture sensor
        double moisture_trigger_cmd_ = 0.0;
        double moisture_adc_state_ = 0.0;

        // GPIO — LDR sensors
        double ldr_trigger_cmd_ = 0.0;
        double ldr_a_ambient_state_ = 0.0;
        double ldr_b_ambient_state_ = 0.0;
        double ldr_a_illuminated_state_ = 0.0;
        double ldr_b_illuminated_state_ = 0.0;
    };

}  // namespace kibisis_hardware
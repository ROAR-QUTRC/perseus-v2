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
     * This class implements the ros2_control SystemInterface for the Kibisis
     * two-wheel differential drive robot. The robot uses GPIO pins on a
     * Raspberry Pi to control motors via a motor control board.
     *
     * Features:
     * - Two-wheel differential drive (left and right wheels)
     * - Velocity command interface for each wheel
     * - Position and velocity state feedback (from encoders)
     * - GPIO-based motor control
     *
     * Robot specifications:
     * - Wheel diameter: 20cm (radius: 0.1m)
     * - Wheel separation: 30cm
     */
    class KibisisSystemHardware final : public hardware_interface::SystemInterface
    {
    public:
        KibisisSystemHardware() = default;
        ~KibisisSystemHardware() override = default;

        // Prevent copying and moving
        KibisisSystemHardware(const KibisisSystemHardware&) = delete;
        KibisisSystemHardware& operator=(const KibisisSystemHardware&) = delete;
        KibisisSystemHardware(KibisisSystemHardware&&) = delete;
        KibisisSystemHardware& operator=(KibisisSystemHardware&&) = delete;

        // ROS 2 Control lifecycle functions
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
        // Constants
        static constexpr double WHEEL_RADIUS = 0.10;      // 20cm diameter
        static constexpr double WHEEL_SEPARATION = 0.30;  // 30cm apart
        static constexpr double RADIANS_PER_REVOLUTION = 2.0 * std::numbers::pi;

        // Joint configuration
        struct JointConfig
        {
            std::string name;
            int gpio_pin_a{-1};
            int gpio_pin_b{-1};
            int encoder_pin_a{-1};
            int encoder_pin_b{-1};
        };
        std::vector<JointConfig> joint_configs_;

        // State storage
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;

        // Command storage
        std::vector<double> hw_commands_;

        // Previous positions for velocity calculation
        std::vector<double> prev_positions_;

        // i2c file descriptor for communicating with the pico
        int _i2c_file = -1;
        std::string _i2c_filename = "/dev/i2c-0";
        const uint8_t _PICO_ADDRESS = 0x01;
        uint8_t _READ_ENCODER_LEFT = 0x04;
        uint8_t _READ_ENCODER_RIGHT = 0x05;

        /**
         * @brief Initialize GPIO pins for motor control
         * @return true if successful, false otherwise
         */
        bool init_i2c();

        /**
         * @brief Cleanup GPIO resources
         */
        void cleanup_i2c();

        /**
         * @brief Set motor speed for a joint
         * @param joint_index Index of the joint (0 = left, 1 = right)
         * @param velocity Target velocity in rad/s
         */
        void set_motor_velocity(size_t joint_index, double velocity);

        /**
         * @brief Read encoder values
         * @param joint_index Index of the joint
         * @return Current position in radians
         */
        double read_encoder(size_t joint_index);
    };

}  // namespace kibisis_hardware

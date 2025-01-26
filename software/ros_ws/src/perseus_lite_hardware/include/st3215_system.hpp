#pragma once

#include <boost/asio.hpp>
#include <hardware_interface/system_interface.hpp>
#include <mutex>
#include <queue>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <span>
#include <string_view>
#include <vector>

namespace perseus_lite_hardware
{

    class ST3215SystemHardware final : public hardware_interface::SystemInterface
    {
    public:
        ST3215SystemHardware() = default;
        ~ST3215SystemHardware() override;

        // Prevent copying and moving
        ST3215SystemHardware(const ST3215SystemHardware&) = delete;
        ST3215SystemHardware& operator=(const ST3215SystemHardware&) = delete;
        ST3215SystemHardware(ST3215SystemHardware&&) = delete;
        ST3215SystemHardware& operator=(ST3215SystemHardware&&) = delete;

        // ROS 2 Control Functions
        [[nodiscard]] hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo& info) override;

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
        static constexpr uint8_t CMD_WRITE_VEL = 0x42;  // Example command code
        static constexpr int16_t MAX_VELOCITY_RPM = 1000;
        static constexpr size_t BUFFER_SIZE = 256;

        [[nodiscard]] bool sendServoCommand(
            uint8_t id, uint8_t cmd, std::span<const uint8_t> data) noexcept;
        void processResponse(std::span<const uint8_t> response) noexcept;
        [[nodiscard]] bool updateServoStates(uint8_t id, size_t index) noexcept;
        void startAsyncRead() noexcept;
        void startAsyncWrite() noexcept;
        void scheduleNextRead() noexcept;
        void scheduleNextWrite() noexcept;

        // Hardware state storage
        std::vector<double> command_positions_;
        std::vector<double> command_speeds_;
        std::vector<double> current_positions_;
        std::vector<double> current_velocities_;
        std::vector<double> current_loads_;
        std::vector<double> temperatures_;
        std::vector<uint8_t> servo_ids_;

        // Communication members
        mutable std::mutex serial_mutex_;
        boost::asio::io_context io_context_;
        boost::asio::serial_port serial_port_{io_context_};
        boost::asio::steady_timer read_timer_{io_context_};
        boost::asio::steady_timer write_timer_{io_context_};
        std::thread io_thread_;
        mutable std::queue<std::vector<uint8_t>> write_queue_;
        std::array<uint8_t, BUFFER_SIZE> read_buffer_;
    };

}  // namespace perseus_lite_hardware
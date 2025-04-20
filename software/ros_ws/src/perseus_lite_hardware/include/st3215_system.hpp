#pragma once

#include <boost/asio.hpp>
#include <hardware_interface/system_interface.hpp>
#include <iomanip>
#include <mutex>
#include <queue>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <span>
#include <sstream>
#include <string_view>
#include <vector>

// Replace #define directives with enum classes
namespace perseus_lite_hardware
{
    // EPROM registers
    enum class ServoEpromRegister : uint8_t
    {
        MODEL_L = 3,
        MODEL_H = 4,
        ID = 5,
        BAUD_RATE = 6,
        MIN_ANGLE_LIMIT_L = 9,
        MIN_ANGLE_LIMIT_H = 10,
        MAX_ANGLE_LIMIT_L = 11,
        MAX_ANGLE_LIMIT_H = 12,
        CW_DEAD = 26,
        CCW_DEAD = 27,
        OFS_L = 31,
        OFS_H = 32,
        MODE = 33
    };

    // SRAM registers
    enum class ServoSramRegister : uint8_t
    {
        TORQUE_ENABLE = 40,
        ACC = 41,
        GOAL_POSITION_L = 42,
        GOAL_POSITION_H = 43,
        GOAL_TIME_L = 44,
        GOAL_TIME_H = 45,
        GOAL_SPEED_L = 46,
        GOAL_SPEED_H = 47,
        TORQUE_LIMIT_L = 48,
        TORQUE_LIMIT_H = 49,
        LOCK = 55,
        PRESENT_POSITION_L = 56,
        PRESENT_POSITION_H = 57,
        PRESENT_SPEED_L = 58,
        PRESENT_SPEED_H = 59,
        PRESENT_LOAD_L = 60,
        PRESENT_LOAD_H = 61,
        PRESENT_VOLTAGE = 62,
        PRESENT_TEMPERATURE = 63,
        MOVING = 66,
        PRESENT_CURRENT_L = 69,
        PRESENT_CURRENT_H = 70
    };

    // Command codes
    enum class ServoCommand : uint8_t
    {
        READ = 0x02,
        WRITE = 0x03
    };

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
        static constexpr double MAX_RPM = 100.0;  // Maximum RPM rating of the ST3215 servo
        static constexpr int16_t MAX_VELOCITY_RPM = 1000;
        static constexpr size_t BUFFER_SIZE = 256;

        // Communication thread control
        std::atomic<bool> _comm_thread_running_{false};
        std::thread _comm_thread_;

        // Thread-safe queues for commands and responses
        mutable std::mutex _state_mutex_;  // Protects servo state updates

        // Servo state tracking
        struct ServoState
        {
            double position{0.0};
            double velocity{0.0};
            double temperature{25.0};
            rclcpp::Time last_update{0, 0, RCL_ROS_TIME};
        };
        std::vector<ServoState> _servo_states_;

        // Thread management methods
        void communicationThread() noexcept;
        void updateServoStates() noexcept;

        // Communication timestamping and timeout
        std::vector<rclcpp::Time> _last_update_times_;
        static constexpr auto SERVO_TIMEOUT = std::chrono::seconds(1);

        bool sendServoCommand(uint8_t id, ServoCommand cmd, std::span<const uint8_t> data) noexcept;
        void processResponse(std::span<const uint8_t> response) noexcept;
        [[nodiscard]] bool updateServoStates(uint8_t id, size_t index) noexcept;
        void startAsyncRead() noexcept;
        void startAsyncWrite() noexcept;
        void scheduleNextRead() noexcept;
        void scheduleNextWrite() noexcept;
        [[nodiscard]] bool verifyCommandInterfaces(
            const hardware_interface::ComponentInfo& joint_info,
            const rclcpp::Logger& logger) const;

        // Hardware state storage
        std::vector<double> _command_speeds_;      // Only velocity commands
        std::vector<double> _current_positions_;   // State feedback
        std::vector<double> _current_velocities_;  // State feedback
        std::vector<double> _temperatures_;        // State feedback
        std::vector<uint8_t> _servo_ids_;

        // Communication members
        mutable std::mutex _serial_mutex_;
        boost::asio::io_context _io_context_;
        boost::asio::serial_port _serial_port_{_io_context_};
        boost::asio::steady_timer _read_timer_{_io_context_};
        boost::asio::steady_timer _write_timer_{_io_context_};
        std::thread _io_thread_;
        mutable std::queue<std::vector<uint8_t>> _write_queue_;
        std::array<uint8_t, BUFFER_SIZE> _read_buffer_;
    };

}  // namespace perseus_lite_hardware
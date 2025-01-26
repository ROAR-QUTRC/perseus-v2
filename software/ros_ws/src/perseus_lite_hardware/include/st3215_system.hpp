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

// EPROM
#define SMS_STS_MODEL_L           3
#define SMS_STS_MODEL_H           4
#define SMS_STS_ID                5
#define SMS_STS_BAUD_RATE         6
#define SMS_STS_MIN_ANGLE_LIMIT_L 9
#define SMS_STS_MIN_ANGLE_LIMIT_H 10
#define SMS_STS_MAX_ANGLE_LIMIT_L 11
#define SMS_STS_MAX_ANGLE_LIMIT_H 12
#define SMS_STS_CW_DEAD           26
#define SMS_STS_CCW_DEAD          27
#define SMS_STS_OFS_L             31
#define SMS_STS_OFS_H             32
#define SMS_STS_MODE              33

// SRAM
#define SMS_STS_TORQUE_ENABLE   40
#define SMS_STS_ACC             41
#define SMS_STS_GOAL_POSITION_L 42
#define SMS_STS_GOAL_POSITION_H 43
#define SMS_STS_GOAL_TIME_L     44
#define SMS_STS_GOAL_TIME_H     45
#define SMS_STS_GOAL_SPEED_L    46
#define SMS_STS_GOAL_SPEED_H    47
#define SMS_STS_TORQUE_LIMIT_L  48
#define SMS_STS_TORQUE_LIMIT_H  49
#define SMS_STS_LOCK            55

#define SMS_STS_PRESENT_POSITION_L  56
#define SMS_STS_PRESENT_POSITION_H  57
#define SMS_STS_PRESENT_SPEED_L     58
#define SMS_STS_PRESENT_SPEED_H     59
#define SMS_STS_PRESENT_LOAD_L      60
#define SMS_STS_PRESENT_LOAD_H      61
#define SMS_STS_PRESENT_VOLTAGE     62
#define SMS_STS_PRESENT_TEMPERATURE 63
#define SMS_STS_MOVING              66
#define SMS_STS_PRESENT_CURRENT_L   69
#define SMS_STS_PRESENT_CURRENT_H   70

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
        static constexpr double MAX_RPM = 100.0;  // Maximum RPM rating of the ST3215 servo
        // st3215 command codes
        static constexpr uint8_t CMD_READ = 0x02;
        static constexpr uint8_t CMD_WRITE = 0x03;
        static constexpr uint8_t REG_GOAL_SPEED = 0x2E;
        static constexpr uint8_t REG_PRESENT_POSITION = 0x38;
        static constexpr uint8_t REG_PRESENT_SPEED = 0x3A;
        static constexpr uint8_t REG_PRESENT_TEMP = 0x3F;

        static constexpr uint8_t REG_MODE = SMS_STS_MODE;                              // 33
        static constexpr uint8_t REG_TORQUE_ENABLE = SMS_STS_TORQUE_ENABLE;            // 40
        static constexpr uint8_t REG_PRESENT_POSITION_L = SMS_STS_PRESENT_POSITION_L;  // 56
        static constexpr uint8_t REG_PRESENT_SPEED_L = SMS_STS_PRESENT_SPEED_L;        // 58
        static constexpr uint8_t REG_GOAL_SPEED_L = SMS_STS_GOAL_SPEED_L;              // 46

        static constexpr int16_t MAX_VELOCITY_RPM = 1000;
        static constexpr size_t BUFFER_SIZE = 256;

        // Communication thread control
        std::atomic<bool> comm_thread_running_{false};
        std::thread comm_thread_;

        // Thread-safe queues for commands and responses
        mutable std::mutex state_mutex_;  // Protects servo state updates

        // Servo state tracking
        struct ServoState
        {
            double position{0.0};
            double velocity{0.0};
            double temperature{25.0};
            rclcpp::Time last_update{0, 0, RCL_ROS_TIME};
        };
        std::vector<ServoState> servo_states_;

        // Thread management methods
        void communicationThread() noexcept;
        void updateServoStates() noexcept;

        // Communication timestamping and timeout
        std::vector<rclcpp::Time> last_update_times_;
        static constexpr auto SERVO_TIMEOUT = std::chrono::seconds(1);

        bool sendServoCommand(uint8_t id, uint8_t cmd, std::span<const uint8_t> data) noexcept;
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
        std::vector<double> command_speeds_;      // Only velocity commands
        std::vector<double> current_positions_;   // State feedback
        std::vector<double> current_velocities_;  // State feedback
        std::vector<double> temperatures_;        // State feedback
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
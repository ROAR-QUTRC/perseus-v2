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

    // Servo error flags
    enum class ServoErrorFlag : uint8_t
    {
        INPUT_VOLTAGE = 0x01,
        ANGLE_LIMIT = 0x02,
        OVERHEATING = 0x04,
        RANGE = 0x08,
        CHECKSUM = 0x10,
        OVERLOAD = 0x20,
        INSTRUCTION = 0x40
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
        // Constants for ST3215 servo specifications
        static constexpr double MAX_RPM = 45.0;                                              // Maximum RPM rating of the ST3215 servo
        static constexpr int16_t MAX_VELOCITY_RPM = 1000;                                    // Maximum velocity value in protocol
        static constexpr int16_t MIN_VELOCITY_RPM = -1000;                                   // Minimum velocity value in protocol
        static constexpr size_t BUFFER_SIZE = 256;                                           // Buffer size for serial communication
        static constexpr uint16_t ENCODER_TICKS_PER_REVOLUTION = 4096;                       // Encoder resolution
        static constexpr double RADIANS_PER_REVOLUTION = 2.0 * M_PI;                         // Radians in one revolution
        static constexpr double SECONDS_PER_MINUTE = 60.0;                                   // For RPM to rad/s conversion
        static constexpr double RPM_TO_RAD_S = RADIANS_PER_REVOLUTION / SECONDS_PER_MINUTE;  // Conversion factor
        static constexpr double RAD_S_TO_RPM = SECONDS_PER_MINUTE / RADIANS_PER_REVOLUTION;  // Inverse conversion

        // Protocol constants
        static constexpr uint8_t PACKET_HEADER_BYTE = 0xFF;     // Packet header byte
        static constexpr size_t PACKET_HEADER_SIZE = 2;         // Two FF bytes in header
        static constexpr size_t PACKET_ID_INDEX = 2;            // Position of ID byte in packet
        static constexpr size_t PACKET_LENGTH_INDEX = 3;        // Position of length byte in packet
        static constexpr size_t PACKET_MIN_SIZE = 4;            // Minimum valid packet size
        static constexpr uint8_t WHEEL_MODE_VALUE = 1;          // Value for wheel mode setting
        static constexpr uint8_t TORQUE_ENABLE_VALUE = 1;       // Value to enable torque
        static constexpr uint8_t PRESENT_POSITION_REG = 0x38;   // Present position register
        static constexpr size_t STATUS_PACKET_DATA_SIZE = 8;    // Expected size of status data
        static constexpr uint16_t SIGN_BIT_MASK = 1 << 15;      // Mask for sign bit in position/velocity
        static constexpr size_t ROOM_TEMPERATURE_CELSIUS = 25;  // Default room temperature

        // Communication timing constants
        static constexpr auto READ_TIMEOUT = std::chrono::milliseconds(10);
        static constexpr auto WRITE_TIMEOUT = std::chrono::milliseconds(1);
        static constexpr auto SERVO_TIMEOUT = std::chrono::seconds(1);
        static constexpr auto COMMAND_DELAY = std::chrono::milliseconds(10);
        static constexpr auto COMMUNICATION_CYCLE_DELAY = std::chrono::milliseconds(20);
        static constexpr auto RESPONSE_TIMEOUT = std::chrono::milliseconds(50);

        // Servo packet indices (relative to data portion)
        static constexpr size_t ERROR_BYTE_INDEX = 0;
        static constexpr size_t POSITION_LOW_BYTE_INDEX = 1;
        static constexpr size_t POSITION_HIGH_BYTE_INDEX = 2;
        static constexpr size_t VELOCITY_LOW_BYTE_INDEX = 3;
        static constexpr size_t VELOCITY_HIGH_BYTE_INDEX = 4;
        static constexpr size_t TEMPERATURE_BYTE_INDEX = 7;

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
            double temperature{ROOM_TEMPERATURE_CELSIUS};
            rclcpp::Time last_update{0, 0, RCL_ROS_TIME};
        };
        std::vector<ServoState> _servo_states_;

        // Thread management methods
        void communicationThread() noexcept;
        void updateServoStates() noexcept;

        // Communication timestamping and timeout
        std::vector<rclcpp::Time> _last_update_times_;

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
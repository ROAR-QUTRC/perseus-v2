#pragma once

#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "simple_networking/client.hpp"

/**
 * @brief ROS2 driver node for the M2M2 LIDAR sensor
 *
 * @details Handles communication with the M2M2 LIDAR sensor over TCP/IP,
 * processes incoming data, and publishes LaserScan and IMU messages.
 * The driver maintains the connection to the sensor and provides configuration
 * capabilities.

 */
class M2M2Lidar : public rclcpp::Node
{
public:
    /**
     * @brief Configuration parameters for the M2M2 LIDAR sensor
     */

    struct sensor_config_t
    {
        double scan_frequency;
        double angular_resolution;
        double min_range;
        double max_range;
    };

    /**
     * @brief Construct a new M2M2Lidar node
     *
     * @param options Node options for ROS2 configuration
     * @throws std::runtime_error If initialization fails
     */
    explicit M2M2Lidar(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // constants
    static constexpr double SCAN_FREQUENCY = 15.0;  // SI unit: Hz
    static constexpr double MIN_RANGE = 0.1;        // SI unit: meters
    static constexpr double MAX_RANGE = 30.0;       // SI unit: meters
    static constexpr float INVALID_DISTANCE = 100000.0f;
    static constexpr float EPSILON = 0.0001f;
    static constexpr std::string_view REQUEST_DELIM{"\r\n\r\n"};
    static constexpr size_t INTERPOLATED_POINTS = 4096;  // Number of points per scan
    // for imu
    static const std::string IMU_COMMAND;

    /**
     * @brief Check if two floating point values are equal within epsilon
     */
    [[nodiscard]] static bool is_within_epsilon(float a, float b, float epsilon = EPSILON);

    // Network communication methods
    [[nodiscard]] bool _send_json_request(const std::string& command, const nlohmann::json& args = nlohmann::json{});
    [[nodiscard]] nlohmann::json _receive_json_response();
    void _send_command(const std::vector<uint8_t>& command);

    // Data processing methods
    [[nodiscard]] std::vector<uint8_t> _decode_base64(const std::string& encoded);
    [[nodiscard]] std::vector<std::tuple<float, float, bool>> _decode_laser_points(const std::string& base_64_encoded);
    [[nodiscard]] std::vector<std::tuple<float, float, bool>> _interpolate_points(const std::vector<std::tuple<float, float, bool>>& points);
    [[nodiscard]] std::vector<uint8_t> _create_config_command(const sensor_config_t& config);

    // ROS setup and operation methods
    void _initialize_publishers();
    void _read_sensor_data();
    void _read_imu_data();

    // Member variables
    std::int32_t _request_id{0};
    std::optional<networking::Client> _client;
    sensor_config_t _config{};

    bool _read_imu{true};  // Default to true (on)

    // ROS2 publishers
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _scan_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_publisher;
    rclcpp::TimerBase::SharedPtr _read_timer;
    rclcpp::TimerBase::SharedPtr _imu_timer;
};

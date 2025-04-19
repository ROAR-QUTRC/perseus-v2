#pragma once

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <numbers>
#include <optional>
#include <ranges>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <stdexcept>
#include <string>
#include <thread>
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
        double scanFrequency;
        double angularResolution;
        double minRange;
        double maxRange;
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
    [[nodiscard]] static bool isWithinEpsilon(float a, float b, float epsilon = EPSILON);

    // Network communication methods
    [[nodiscard]] bool _sendJsonRequest(const std::string& command, const nlohmann::json& args = nullptr);
    [[nodiscard]] nlohmann::json _receiveJsonResponse();
    void _sendCommand(const std::vector<uint8_t>& command);

    // Data processing methods
    [[nodiscard]] std::vector<uint8_t> _decodeBase64(const std::string& encoded);
    [[nodiscard]] std::vector<std::tuple<float, float, bool>> _decodeLaserPoints(const std::string& base64Encoded);
    [[nodiscard]] std::vector<std::tuple<float, float, bool>> _interpolatePoints(const std::vector<std::tuple<float, float, bool>>& points);
    [[nodiscard]] std::vector<uint8_t> _createConfigCommand(const sensor_config_t& config);

    // ROS setup and operation methods
    void _initializePublishers();
    void _readSensorData();
    void _readImuData();

    // Member variables
    std::int32_t _requestId{0};
    std::optional<networking::Client> _client;
    sensor_config_t _config{};

    bool _readImu{true};  // Default to true (on)

    // ROS2 publishers
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _scanPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imuPublisher;
    rclcpp::TimerBase::SharedPtr _readTimer;
    rclcpp::TimerBase::SharedPtr _imuTimer;
};

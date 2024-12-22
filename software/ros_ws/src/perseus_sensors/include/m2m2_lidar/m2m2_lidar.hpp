#pragma once

#include <chrono>
#include <iomanip>
#include <nlohmann/json.hpp>  // JSON parsing
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <thread>
#include <vector>

/**
 * @brief Main class for interfacing with the M2M2 Lidar sensor
 *
 * This class handles both the network communication and protocol parsing
 * for the M2M2 Lidar. While this combines two responsibilities, they are
 * tightly coupled for this specific sensor and the data flow between
 * them is well-defined.
 */
class M2M2Lidar : public rclcpp::Node
{
public:
    struct SensorConfig
    {
        double scanFrequency;
        double angularResolution;
        double minRange;
        double maxRange;
    };

    explicit M2M2Lidar(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~M2M2Lidar();  // destructor virtual because class inherits from rclcpp::Node

private:
    std::vector<uint8_t> _decodeBase64(const std::string& encoded);

    std::vector<std::tuple<float, float, bool>> _decodeLaserPoints(const std::string& base64_encoded);

    bool connectToSensor(const std::string& host, int port);

    bool _sendJsonRequest(const std::string& command, const nlohmann::json& args = nullptr);
    nlohmann::json _receiveJsonResponse();

    // Network communication
    void _sendCommand(const std::vector<uint8_t>& command);
    std::optional<std::vector<uint8_t>> _receiveData();

    // Protocol handling
    std::vector<uint8_t> _createConfigCommand(const SensorConfig& config);
    // std::optional<sensor_msgs::msg::LaserScan> _parseLaserScanData(const std::vector<uint8_t>& data);
    // std::optional<sensor_msgs::msg::Imu> _parseImuData(const std::vector<uint8_t>& data);

    // ROS setup
    void _initializePublishers();
    void _publishData();
    void _readSensorData();

    // Member variables
    static constexpr const char* REQUEST_DELIM = "\r\n\r\n";
    int _requestId;
    std::string _sensorAddress;
    uint16_t _sensorPort;
    int _socket;
    bool _isConnected;
    SensorConfig _config;

    // ROS2 publishers
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _scanPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imuPublisher;
    rclcpp::TimerBase::SharedPtr _readTimer;
};

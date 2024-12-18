#pragma once

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
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
    /**
     * @brief Configuration parameters for the M2M2 Lidar
     *
     * Groups all sensor-specific configuration parameters in one place,
     * making it easier to modify settings and extend functionality
     */
    struct SensorConfig
    {
        double scanFrequency;
        double angularResolution;
        double minRange;
        double maxRange;
    };
    /**
     * @brief Constructor for the M2M2 Lidar node
     * @param options ROS2 node options for configuration
     */
    explicit M2M2Lidar(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief Clean up
     */
    ~M2M2Lidar();

private:
    // Network communication
    void _sendCommand(const std::vector<uint8_t>& command);
    std::optional<std::vector<uint8_t>> _receiveData();

    // Protocol handling
    std::vector<uint8_t> _createConfigCommand(const SensorConfig& config);
    std::optional<sensor_msgs::msg::LaserScan> _parseLaserScanData(const std::vector<uint8_t>& data);
    std::optional<sensor_msgs::msg::Imu> _parseImuData(const std::vector<uint8_t>& data);

    // ROS setup
    void _initializePublishers();
    void _publishData();
    void _readSensorData();

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
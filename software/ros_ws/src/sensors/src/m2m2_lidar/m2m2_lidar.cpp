/**
 * @file m2m2_lidar.cpp
 * @brief ROS2 driver for M2M2 LIDAR sensor
 *
 * @details Expected sequence of flow:
 * 1. Node initialisation:
 *    - Loads ROS params (IP, port, topics, frame_id) - likely to come from a launch file in future
 *    - Establishes TCP socket connection to sensor
 *    - Sends initial configuration command - need to test if this is necessary
 *    - Creates publishers for scan and IMU data via ROS topic
 *
 * 2. Main operation loop (100ms intervals):
 *    - Reads raw data packets from sensor
 *    - Parses into LaserScan messages:
 *      + Single 360° sweep per scan
 *      + Each scan timestamped
 *      + Points evenly spaced with time_increment
 *    - Parses IMU data if present
 *    - Publishes to respective topics
 *
 * 3. Cleanup on shutdown:
 *    - Closes socket connection
 *    - Releases resources
 *
 * This is under construction but hopefully helpful as multiple people are collaborating on this.
 *
 * @note Hardware to Laserscan generation:
 * The driver is going to take a single 360 degree sweep as a "scan".
 * Each scan is timestamped. Each datapoint in the scan is not timestamped.
 * Individual data points in the scan are not timestamped but are assumed to be evenly spaced and have a time_increment value.
 * scan_time is the time for a one complete 360 degree sweep.
 *
 * *
 * @see sensor_msgs::msg::LaserScan
 * @see sensor_msgs::msg::Imu
 */

#include "m2m2_lidar/m2m2_lidar.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

M2M2Lidar::M2M2Lidar(const rclcpp::NodeOptions& options)
    : Node("m2m2_lidar", options)
{
    // Parameter setup
    this->declare_parameter("sensor_ip", "192.168.1.100");
    this->declare_parameter("sensor_port", 8080);
    this->declare_parameter("frame_id", "lidar_frame");
    this->declare_parameter("scan_topic", "scan");
    this->declare_parameter("imu_topic", "imu");

    // TODO: Error handling (exceptions)
    _sensorAddress = this->get_parameter("sensor_ip").as_string();
    _sensorPort = static_cast<uint16_t>(
        this->get_parameter("sensor_port").as_int());

    // TODO: Replace with network library
    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(_sensorPort);

    if (inet_pton(AF_INET, _sensorAddress.c_str(), &serverAddr.sin_addr) <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid address");
        // TODO: Throw exception
    }

    _socket = socket(AF_INET, SOCK_STREAM, 0);
    if (_socket < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
        // TODO: Throw exception
    }
    if (connect(_socket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Connection failed");
        close(_socket);
        // TODO: Throw exception
    }

    // Create and send initial configuration
    SensorConfig defaultConfig{
        .scanFrequency = 15.0,
        .angularResolution = 0.25,
        .minRange = 0.1,
        .maxRange = 30.0};
    // TODO: Exception handling
    _sendCommand(_createConfigCommand(defaultConfig));

    // Start the data reading timer
    _readTimer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&M2M2Lidar::_readSensorData, this));
    _initializePublishers();
}

M2M2Lidar::~M2M2Lidar()
{
    close(_socket);
}

void M2M2Lidar::_initializePublishers()
{
    const auto qos = rclcpp::QoS(10);
    _scanPublisher = this->create_publisher<sensor_msgs::msg::LaserScan>(
        this->get_parameter("scan_topic").as_string(), qos);
    _imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>(
        this->get_parameter("imu_topic").as_string(), qos);
}

/**
 * @brief Creates a configuration command packet for the M2M2 LIDAR sensor.
 *
 * @details Constructs a byte sequence following the sensor's protocol format for
 * configuration commands. Note that this function may become deprecated as hardware
 * testing progresses - the m2m2 LIDAR's default configuration may be sufficient
 * without manual configuration. Testing will confirm or not.
 *
 */
std::vector<uint8_t> M2M2Lidar::_createConfigCommand(const SensorConfig& config)
{
    // Protocol-related constants
    static constexpr uint8_t PROTOCOL_HEADER = 0xAA;
    static constexpr uint8_t PROTOCOL_FOOTER = 0x55;

    std::vector<uint8_t> command;
    command.push_back(PROTOCOL_HEADER);

    // Add command type (example: 0x01 for configuration)
    command.push_back(0x01);

    // Add configuration parameters if needed - this might go-away post hardware testing
    // Note: This is a simplified example - you'll need to match your actual protocol
    command.push_back(static_cast<uint8_t>(config.scanFrequency));
    command.push_back(static_cast<uint8_t>(config.angularResolution * 100));

    // Add footer
    command.push_back(PROTOCOL_FOOTER);

    return command;
}

void M2M2Lidar::_sendCommand(const std::vector<uint8_t>& command)
{
    ssize_t bytesSent = send(_socket, command.data(), command.size(), 0);
    if (bytesSent != static_cast<ssize_t>(command.size()))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send complete command");
        // TODO: Throw exception
    }
}

void M2M2Lidar::_readSensorData()
{
    // Buffer for receiving data
    std::vector<uint8_t> buffer(1024);  // Adjust size based on your sensor's needs
    ssize_t bytesRead = recv(_socket, buffer.data(), buffer.size(), 0);

    if (bytesRead <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading from sensor");
        return;
    }

    // Resize buffer to actual data received
    buffer.resize(bytesRead);

    // Note: These are placeholder implementations - we need to implement
    // proper parsing based on m2m2 protocol
    // TODO: Error handling
    auto maybeScan = _parseLaserScanData(buffer);
    if (maybeScan)
    {
        maybeScan->header.stamp = this->now();
        maybeScan->header.frame_id =
            this->get_parameter("frame_id").as_string();
        _scanPublisher->publish(*maybeScan);
    }

    auto maybeImu = _parseImuData(buffer);
    if (maybeImu)
    {
        maybeImu->header.stamp = this->now();
        maybeImu->header.frame_id =
            this->get_parameter("frame_id").as_string();
        _imuPublisher->publish(*maybeImu);
    }
}

std::optional<sensor_msgs::msg::LaserScan> M2M2Lidar::_parseLaserScanData([[maybe_unused]] const std::vector<uint8_t>& data)
{
    sensor_msgs::msg::LaserScan scan{};

    // TODO: Implement

    return scan;
}

std::optional<sensor_msgs::msg::Imu> M2M2Lidar::_parseImuData([[maybe_unused]] const std::vector<uint8_t>& data)
{
    sensor_msgs::msg::Imu imu{};

    // TODO: Implement

    return imu;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<M2M2Lidar>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        // TODO: Maybe get a logger with rclcpp::get_logger() and log error, but that should be handled internally to the node anyway
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
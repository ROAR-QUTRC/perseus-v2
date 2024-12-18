// File: src/m2m2_lidar/m2m2_lidar.cpp

#include "sensors/m2m2_lidar/m2m2_lidar.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace
{
    // Helper functions that don't need class access
    bool validateChecksum(const std::vector<uint8_t>& data)
    {
        // Implementation of checksum validation
        return true;  // Placeholder
    }

    uint16_t calculateChecksum(const std::vector<uint8_t>& data)
    {
        // Implementation of checksum calculation
        return 0;  // Placeholder
    }
}  // anonymous namespace

namespace sensors
{
    namespace lidar
    {
        namespace m2m2
        {

            Lidar::Lidar(const rclcpp::NodeOptions& options)
                : Node("m2m2_lidar", options), _socket(-1), _isConnected(false)
            {
                // Declare and get parameters
                this->declare_parameter("sensor_ip", "192.168.1.100");
                this->declare_parameter("sensor_port", 2000);
                this->declare_parameter("frame_id", "lidar_frame");
                this->declare_parameter("scan_topic", "scan");
                this->declare_parameter("imu_topic", "imu");

                _sensorAddress = this->get_parameter("sensor_ip").as_string();
                _sensorPort = static_cast<uint16_t>(
                    this->get_parameter("sensor_port").as_int());

                initializePublishers();
            }

            Lidar::~Lidar()
            {
                disconnect();
            }

            bool Lidar::initialize()
            {
                if (!connectToSensor())
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to connect to sensor");
                    return false;
                }

                // Create and send initial configuration
                SensorConfig defaultConfig{
                    .scanFrequency = 15.0,
                    .angularResolution = 0.25,
                    .minRange = 0.1,
                    .maxRange = 30.0};

                auto configCommand = createConfigCommand(defaultConfig);
                if (!sendCommand(configCommand))
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to configure sensor");
                    return false;
                }

                // Start the data reading timer
                _readTimer = this->create_wall_timer(
                    std::chrono::milliseconds(100),
                    std::bind(&Lidar::readSensorData, this));

                return true;
            }

            void Lidar::initializePublishers()
            {
                const auto qos = rclcpp::QoS(10);
                _scanPublisher = this->create_publisher<sensor_msgs::msg::LaserScan>(
                    this->get_parameter("scan_topic").as_string(), qos);
                _imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>(
                    this->get_parameter("imu_topic").as_string(), qos);
            }

            bool Lidar::connectToSensor()
            {
                _socket = socket(AF_INET, SOCK_STREAM, 0);
                if (_socket < 0)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
                    return false;
                }

                struct sockaddr_in serverAddr;
                serverAddr.sin_family = AF_INET;
                serverAddr.sin_port = htons(_sensorPort);

                if (inet_pton(AF_INET, _sensorAddress.c_str(), &serverAddr.sin_addr) <= 0)
                {
                    RCLCPP_ERROR(this->get_logger(), "Invalid address");
                    close(_socket);
                    _socket = -1;
                    return false;
                }

                if (connect(_socket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0)
                {
                    RCLCPP_ERROR(this->get_logger(), "Connection failed");
                    close(_socket);
                    _socket = -1;
                    return false;
                }

                _isConnected = true;
                return true;
            }

            void Lidar::disconnect()
            {
                if (_socket != -1)
                {
                    close(_socket);
                    _socket = -1;
                }
                _isConnected = false;
            }

            std::vector<uint8_t> Lidar::createConfigCommand(const SensorConfig& config)
            {
                std::vector<uint8_t> command;
                command.push_back(PROTOCOL_HEADER);

                // Add command type (example: 0x01 for configuration)
                command.push_back(0x01);

                // Add configuration parameters
                // Note: This is a simplified example - you'll need to match your actual protocol
                command.push_back(static_cast<uint8_t>(config.scanFrequency));
                command.push_back(static_cast<uint8_t>(config.angularResolution * 100));

                // Add footer
                command.push_back(PROTOCOL_FOOTER);

                return command;
            }

            bool Lidar::sendCommand(const std::vector<uint8_t>& command)
            {
                if (!_isConnected)
                {
                    RCLCPP_ERROR(this->get_logger(), "Not connected to sensor");
                    return false;
                }

                ssize_t bytesSent = send(_socket, command.data(), command.size(), 0);
                if (bytesSent != static_cast<ssize_t>(command.size()))
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to send complete command");
                    return false;
                }

                return true;
            }

            void Lidar::readSensorData()
            {
                if (!_isConnected)
                {
                    RCLCPP_ERROR_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        1000,  // Throttle to once per second
                        "Not connected to sensor");
                    return;
                }

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
                auto maybeScan = parseLaserScanData(buffer);
                if (maybeScan)
                {
                    maybeScan->header.stamp = this->now();
                    maybeScan->header.frame_id =
                        this->get_parameter("frame_id").as_string();
                    _scanPublisher->publish(*maybeScan);
                }

                auto maybeImu = parseImuData(buffer);
                if (maybeImu)
                {
                    maybeImu->header.stamp = this->now();
                    maybeImu->header.frame_id =
                        this->get_parameter("frame_id").as_string();
                    _imuPublisher->publish(*maybeImu);
                }
            }

            // We also need to implement the parsing methods
            std::optional<sensor_msgs::msg::LaserScan> Lidar::parseLaserScanData(
                const std::vector<uint8_t>& data)
            {
                sensor_msgs::msg::LaserScan scan;
                // This is a placeholder implementation
                // You'll need to implement actual parsing based on your sensor's protocol
                scan.angle_min = -3.14159f;
                scan.angle_max = 3.14159f;
                scan.angle_increment = 0.01745f;
                scan.time_increment = 0.0001f;
                scan.scan_time = 0.1f;
                scan.range_min = 0.1f;
                scan.range_max = 30.0f;

                return scan;
            }

            std::optional<sensor_msgs::msg::Imu> Lidar::parseImuData(
                const std::vector<uint8_t>& data)
            {
                sensor_msgs::msg::Imu imu;
                // This is a placeholder implementation
                // You'll need to implement actual parsing based on your sensor's protocol
                imu.orientation_covariance[0] = -1;  // Orientation not provided
                imu.angular_velocity_covariance[0] = 0.1;
                imu.linear_acceleration_covariance[0] = 0.1;

                return imu;
            }

        }
    }
}  // namespace sensors::lidar::m2m2

// the main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Initialise the m2m2 lidar node
    auto lidar_node = std::make_shared<sensors::lidar::m2m2::Lidar>();

    if (!lidar_node->initialize())
    {
        RCLCPP_ERROR(lidar_node->get_logger(), "Failed to initialise m2m2 lidar");
        return 1;
    }

    // spin the node
    rclcpp::spin(lidar_node);

    rclcpp::shutdown();
    return 0;
}
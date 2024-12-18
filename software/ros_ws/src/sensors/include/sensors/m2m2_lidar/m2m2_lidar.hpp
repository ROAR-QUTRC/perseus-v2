// File: include/sensors/m2m2_lidar/m2m2_lidar.hpp

#pragma once

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <vector>

namespace sensors
{
    namespace lidar
    {
        namespace m2m2
        {

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
             * @brief Main class for interfacing with the M2M2 Lidar sensor
             *
             * This class handles both the network communication and protocol parsing
             * for the M2M2 Lidar. While this combines two responsibilities, they are
             * tightly coupled for this specific sensor and the data flow between
             * them is well-defined.
             */
            class Lidar : public rclcpp::Node
            {
            public:
                /**
                 * @brief Constructor for the M2M2 Lidar node
                 * @param options ROS2 node options for configuration
                 */
                explicit Lidar(
                    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

                /**
                 * @brief Destructor ensures proper cleanup of resources
                 */
                ~Lidar();

                /**
                 * @brief Initialize the lidar connection and start data processing
                 * @return true if initialization successful
                 */
                bool initialize();

            private:
                // Network communication methods
                bool connectToSensor();
                bool sendCommand(const std::vector<uint8_t>& command);
                std::optional<std::vector<uint8_t>> receiveData();
                void disconnect();

                // Protocol handling methods
                std::vector<uint8_t> createConfigCommand(const SensorConfig& config);
                std::optional<sensor_msgs::msg::LaserScan> parseLaserScanData(
                    const std::vector<uint8_t>& data);
                std::optional<sensor_msgs::msg::Imu> parseImuData(
                    const std::vector<uint8_t>& data);

                // ROS2 interface methods
                void initializePublishers();
                void publishData();
                void readSensorData();

                // Member variables
                std::string _sensorAddress;
                uint16_t _sensorPort;
                int _socket;
                bool _isConnected;
                SensorConfig _config;

                // Protocol-related constants
                static constexpr uint8_t PROTOCOL_HEADER = 0xAA;
                static constexpr uint8_t PROTOCOL_FOOTER = 0x55;

                // ROS2 publishers
                rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _scanPublisher;
                rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imuPublisher;
                rclcpp::TimerBase::SharedPtr _readTimer;
            };

        }
    }
}  // namespace sensors::lidar::m2m2
#pragma once

#include <cstdint>
#include <memory>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace sensors
{
    class Protocol 
    {
    public:
        /**
         * @brief Constructor for the Protocol class
         */
        Protocol();
        
        /**
         * @brief Destructor
         */
        ~Protocol();

        /**
         * @brief Parse raw sensor data into LaserScan message
         * @param data Raw data buffer from sensor
         * @param length Length of data buffer
         * @return Parsed LaserScan message
         */
        sensor_msgs::msg::LaserScan parseLaserScanData(const uint8_t* data, size_t length);

        /**
         * @brief Parse raw sensor data into IMU message
         * @param data Raw data buffer from sensor
         * @param length Length of data buffer
         * @return Parsed IMU message
         */
        sensor_msgs::msg::Imu parseImuData(const uint8_t* data, size_t length);

        /**
         * @brief Create command buffer for sensor configuration
         * @param buffer Output buffer for command
         * @param maxLength Maximum length of buffer
         * @return Length of command written to buffer
         */
        size_t createConfigCommand(uint8_t* buffer, size_t maxLength);

    private:
        // Internal protocol-specific constants
        static constexpr uint8_t PROTOCOL_HEADER = 0xAA;
        static constexpr uint8_t PROTOCOL_FOOTER = 0x55;
};

} // namespace sensors

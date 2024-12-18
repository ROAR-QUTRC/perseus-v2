#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace sensors
{
    class Communication 
    {
    public:
        /**
         * @brief Constructor for the Communication class
         * @param address IP address of the M2M2 Lidar sensor
         * @param port Port number for communication
         */
        explicit Communication(const std::string& address, uint16_t port);
        
        /**
         * @brief Destructor ensuring proper cleanup of resources
         */
        ~Communication();

        /**
         * @brief Initialize the communication with the sensor
         * @return true if initialization successful, false otherwise
         */
        bool initialize();

        /**
         * @brief Send a command to the sensor
         * @param command Command buffer to send
         * @param length Length of the command
         * @return true if send successful, false otherwise
         */
        bool sendCommand(const uint8_t* command, size_t length);

        /**
         * @brief Receive data from the sensor
         * @param buffer Buffer to store received data
         * @param maxLength Maximum length of data to receive
         * @return Number of bytes received, or -1 on error
         */
        ssize_t receiveData(uint8_t* buffer, size_t maxLength);

    private:
        std::string _address;
        uint16_t _port;
        int _socket;
};

} // namespace sensors

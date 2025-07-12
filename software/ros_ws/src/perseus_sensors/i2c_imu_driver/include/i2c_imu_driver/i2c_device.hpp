#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "fd_wrapper.hpp"

namespace i2c_imu_driver
{

    /**
     * @brief Low-level I2C device communication wrapper
     *
     * This class provides a clean interface for I2C communication using Linux I2C API.
     * It manages the I2C device file descriptor and provides methods for device detection,
     * register reading/writing, and proper error handling.
     */
    class I2cDevice
    {
    public:
        /**
         * @brief Construct a new I2cDevice object
         *
         * @param bus_path Path to the I2C bus device file (e.g., "/dev/i2c-7")
         * @param device_address 7-bit I2C device address
         */
        I2cDevice(const std::string& bus_path, uint8_t device_address);

        /**
         * @brief Initialize the I2C device connection
         *
         * @return true if initialization succeeded, false otherwise
         */
        bool initialize();

        /**
         * @brief Check if the device is connected and responsive
         *
         * @return true if device is connected, false otherwise
         */
        bool isConnected() const;

        /**
         * @brief Read a single byte from a register
         *
         * @param reg_address Register address to read from
         * @param timeout_ms Timeout in milliseconds for the operation
         * @return Register value if successful, std::nullopt otherwise
         */
        std::optional<uint8_t> readRegister(uint8_t reg_address,
                                            std::chrono::milliseconds timeout_ms = std::chrono::milliseconds(100));

        /**
         * @brief Write a single byte to a register
         *
         * @param reg_address Register address to write to
         * @param value Value to write
         * @param timeout_ms Timeout in milliseconds for the operation
         * @return true if write succeeded, false otherwise
         */
        bool writeRegister(uint8_t reg_address, uint8_t value,
                           std::chrono::milliseconds timeout_ms = std::chrono::milliseconds(100));

        /**
         * @brief Read multiple bytes from consecutive registers
         *
         * @param reg_address Starting register address
         * @param buffer Buffer to store the read data
         * @param length Number of bytes to read
         * @param timeout_ms Timeout in milliseconds for the operation
         * @return true if read succeeded, false otherwise
         */
        bool readRegisters(uint8_t reg_address, uint8_t* buffer, size_t length,
                           std::chrono::milliseconds timeout_ms = std::chrono::milliseconds(100));

        /**
         * @brief Write multiple bytes to consecutive registers
         *
         * @param reg_address Starting register address
         * @param buffer Buffer containing data to write
         * @param length Number of bytes to write
         * @param timeout_ms Timeout in milliseconds for the operation
         * @return true if write succeeded, false otherwise
         */
        bool writeRegisters(uint8_t reg_address, const uint8_t* buffer, size_t length,
                            std::chrono::milliseconds timeout_ms = std::chrono::milliseconds(100));

        /**
         * @brief Get the device address
         *
         * @return uint8_t Device address
         */
        uint8_t getDeviceAddress() const { return _device_address; }

        /**
         * @brief Get the bus path
         *
         * @return const std::string& Bus path
         */
        const std::string& getBusPath() const { return _bus_path; }

    private:
        /**
         * @brief Perform device detection by attempting to read from device
         *
         * @return true if device detected, false otherwise
         */
        bool _performDeviceDetection();

        /**
         * @brief Set the I2C slave address for communication
         *
         * @return true if successful, false otherwise
         */
        bool _setSlaveAddress();

        std::string _bus_path;
        uint8_t _device_address;
        FdWrapper _i2c_fd;
        bool _is_initialized{false};
    };

}  // namespace i2c_imu_driver
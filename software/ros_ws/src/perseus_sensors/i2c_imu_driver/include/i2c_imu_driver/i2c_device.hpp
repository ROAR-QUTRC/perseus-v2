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
         * Opens the I2C bus and configures the slave address. The object is fully
         * initialized upon successful construction following RAII principles.
         *
         * @param bus_path Path to the I2C bus device file (e.g., "/dev/i2c-7")
         * @param device_address 7-bit I2C device address
         * @throws std::runtime_error if I2C bus cannot be opened or configured
         */
        I2cDevice(const std::string& bus_path, uint8_t device_address);

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
         * @return Register value if successful, std::nullopt otherwise
         *
         * @note Uses hardware I2C timeout (configured at bus level, typically 1-2 seconds)
         */
        std::optional<uint8_t> readRegister(uint8_t reg_address);

        /**
         * @brief Write a single byte to a register
         *
         * @param reg_address Register address to write to
         * @param value Value to write
         * @return true if write succeeded, false otherwise
         *
         * @note Uses hardware I2C timeout (configured at bus level, typically 1-2 seconds)
         */
        bool writeRegister(uint8_t reg_address, uint8_t value);

        /**
         * @brief Read multiple bytes from consecutive registers
         *
         * Performs an I2C combined transaction: write register address, then read data.
         *
         * @param reg_address Starting register address
         * @param buffer Output buffer (must be at least @p length bytes)
         * @param length Number of bytes to read (maximum: 255)
         * @return true if read succeeded, false if:
         *         - Device not connected
         *         - Buffer is null
         *         - Length is 0 or > 255
         *         - I2C transaction failed
         *
         * @pre buffer != nullptr
         * @pre length > 0 && length <= 255
         * @pre buffer has at least @p length bytes allocated
         *
         * @note Uses hardware I2C timeout (configured at bus level, typically 1-2 seconds)
         * @threadsafe NO - Must not be called from multiple threads concurrently
         */
        bool readRegisters(uint8_t reg_address, uint8_t* buffer, size_t length);

        /**
         * @brief Write multiple bytes to consecutive registers
         *
         * @param reg_address Starting register address
         * @param buffer Buffer containing data to write (must be at least @p length bytes)
         * @param length Number of bytes to write (maximum: 255)
         * @return true if write succeeded, false otherwise
         *
         * @pre buffer != nullptr
         * @pre length > 0 && length <= 255
         *
         * @note Uses hardware I2C timeout (configured at bus level, typically 1-2 seconds)
         * @threadsafe NO - Must not be called from multiple threads concurrently
         */
        bool writeRegisters(uint8_t reg_address, const uint8_t* buffer, size_t length);

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
        std::string _bus_path;
        uint8_t _device_address;
        FdWrapper _i2c_fd;
    };

}  // namespace i2c_imu_driver
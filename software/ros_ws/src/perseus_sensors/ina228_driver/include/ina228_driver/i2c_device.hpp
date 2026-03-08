#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "fd_wrapper.hpp"

namespace ina228_driver
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
         * Opens the I2C bus and configures the slave address.
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
        bool is_connected() const;

        /**
         * @brief Read a single byte from a register
         *
         * @param reg_address Register address to read from
         * @return Register value if successful, std::nullopt otherwise
         */
        std::optional<uint8_t> read_register(uint8_t reg_address);

        /**
         * @brief Write a single byte to a register
         *
         * @param reg_address Register address to write to
         * @param value Value to write
         * @return true if write succeeded, false otherwise
         */
        bool write_register(uint8_t reg_address, uint8_t value);

        /**
         * @brief Read multiple bytes from consecutive registers
         *
         * @param reg_address Starting register address
         * @param buffer Output buffer (must be at least @p length bytes)
         * @param length Number of bytes to read (maximum: 255)
         * @return true if read succeeded, false otherwise
         */
        bool read_registers(uint8_t reg_address, uint8_t* buffer, size_t length);

        /**
         * @brief Write multiple bytes to consecutive registers
         *
         * @param reg_address Starting register address
         * @param buffer Buffer containing data to write
         * @param length Number of bytes to write (maximum: 255)
         * @return true if write succeeded, false otherwise
         */
        bool write_registers(uint8_t reg_address, const uint8_t* buffer, size_t length);

        uint8_t get_device_address() const { return _device_address; }

        const std::string& get_bus_path() const { return _bus_path; }

    private:
        std::string _bus_path;
        uint8_t _device_address;
        FdWrapper _i2c_fd;
    };

}  // namespace ina228_driver

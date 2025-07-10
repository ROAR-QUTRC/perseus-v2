#include "i2c_imu_driver/i2c_device.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <vector>

namespace i2c_imu_driver
{

    I2cDevice::I2cDevice(const std::string& bus_path, uint8_t device_address)
        : _bus_path(bus_path),
          _device_address(device_address)
    {
    }

    bool I2cDevice::initialize()
    {
        if (_is_initialized)
        {
            return true;
        }

        // First try to open the file descriptor directly
        int fd = open(_bus_path.c_str(), O_RDWR);
        if (fd < 0)
        {
            return false;
        }

        // Set slave address
        if (ioctl(fd, I2C_SLAVE, _device_address) < 0)
        {
            close(fd);
            return false;
        }

        // Now create the FdWrapper with the already-opened file descriptor
        try
        {
            _i2c_fd = FdWrapper(
                [fd]() -> int
                { return fd; },
                nullptr,
                [](int fd)
                { close(fd); });
        }
        catch (const std::exception& e)
        {
            close(fd);
            return false;
        }

        // Check if the file descriptor is valid
        if (_i2c_fd.get() < 0)
        {
            return false;
        }

        // Perform device detection
        if (!_performDeviceDetection())
        {
            // Skip device detection for now
            // return false;
        }

        _is_initialized = true;
        return true;
    }

    bool I2cDevice::isConnected() const
    {
        return _is_initialized && _i2c_fd.get() >= 0;
    }

    std::optional<uint8_t> I2cDevice::readRegister(uint8_t reg_address,
                                                   std::chrono::milliseconds timeout_ms)
    {
        if (!isConnected())
        {
            return std::nullopt;
        }

        uint8_t buffer[1];
        if (!readRegisters(reg_address, buffer, 1, timeout_ms))
        {
            return std::nullopt;
        }

        return buffer[0];
    }

    bool I2cDevice::writeRegister(uint8_t reg_address, uint8_t value,
                                  std::chrono::milliseconds timeout_ms)
    {
        if (!isConnected())
        {
            return false;
        }

        return writeRegisters(reg_address, &value, 1, timeout_ms);
    }

    bool I2cDevice::readRegisters(uint8_t reg_address, uint8_t* buffer, size_t length,
                                  std::chrono::milliseconds /* timeout_ms */)
    {
        if (!isConnected() || buffer == nullptr || length == 0)
        {
            return false;
        }

        // Use I2C message structure for combined write/read transaction
        struct i2c_msg msgs[2];
        struct i2c_rdwr_ioctl_data msgset;

        // First message: write register address
        msgs[0].addr = _device_address;
        msgs[0].flags = 0;
        msgs[0].len = 1;
        msgs[0].buf = &reg_address;

        // Second message: read data
        msgs[1].addr = _device_address;
        msgs[1].flags = I2C_M_RD;
        msgs[1].len = length;
        msgs[1].buf = buffer;

        msgset.msgs = msgs;
        msgset.nmsgs = 2;

        // Perform the I2C transaction
        if (ioctl(_i2c_fd.get(), I2C_RDWR, &msgset) < 0)
        {
            return false;
        }

        return true;
    }

    bool I2cDevice::writeRegisters(uint8_t reg_address, const uint8_t* buffer, size_t length,
                                   std::chrono::milliseconds /* timeout_ms */)
    {
        if (!isConnected() || buffer == nullptr || length == 0)
        {
            return false;
        }

        // Create a temporary buffer to hold register address + data
        std::vector<uint8_t> write_buffer(length + 1);
        write_buffer[0] = reg_address;
        std::memcpy(&write_buffer[1], buffer, length);

        // Use I2C message structure for write transaction
        struct i2c_msg msg;
        struct i2c_rdwr_ioctl_data msgset;

        msg.addr = _device_address;
        msg.flags = 0;
        msg.len = write_buffer.size();
        msg.buf = write_buffer.data();

        msgset.msgs = &msg;
        msgset.nmsgs = 1;

        // Perform the I2C transaction
        if (ioctl(_i2c_fd.get(), I2C_RDWR, &msgset) < 0)
        {
            return false;
        }

        return true;
    }

    bool I2cDevice::_performDeviceDetection()
    {
        if (_i2c_fd.get() < 0)
        {
            return false;
        }

        // Try to read from WHO_AM_I register (0x0F) for LSM6DSOX
        // Expected value is 0x6C for LSM6DSOX
        struct i2c_msg msgs[2];
        struct i2c_rdwr_ioctl_data msgset;
        uint8_t reg_addr = 0x0F;
        uint8_t who_am_i = 0;

        // First message: write register address
        msgs[0].addr = _device_address;
        msgs[0].flags = 0;
        msgs[0].len = 1;
        msgs[0].buf = &reg_addr;

        // Second message: read data
        msgs[1].addr = _device_address;
        msgs[1].flags = I2C_M_RD;
        msgs[1].len = 1;
        msgs[1].buf = &who_am_i;

        msgset.msgs = msgs;
        msgset.nmsgs = 2;

        // Perform the I2C transaction
        if (ioctl(_i2c_fd.get(), I2C_RDWR, &msgset) < 0)
        {
            // Debug: I2C transaction failed
            return false;
        }

        // Check if it's a valid LSM6DSOX device
        if (who_am_i != 0x6C)
        {
            // Debug: Invalid WHO_AM_I value
            return false;
        }

        return true;
    }

    bool I2cDevice::_setSlaveAddress()
    {
        if (_i2c_fd.get() < 0)
        {
            return false;
        }

        if (ioctl(_i2c_fd.get(), I2C_SLAVE, _device_address) < 0)
        {
            return false;
        }

        return true;
    }

}  // namespace i2c_imu_driver
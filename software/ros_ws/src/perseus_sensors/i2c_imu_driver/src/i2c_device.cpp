#include "i2c_imu_driver/i2c_device.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <optional>
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

        // Create file descriptor wrapper with proper open and close functions
        _i2c_fd = FdWrapper(
            [this]() -> int
            {
                return open(_bus_path.c_str(), O_RDWR);
            },
            [this](int fd)
            {
                // Configure the I2C device
                if (ioctl(fd, I2C_SLAVE, _device_address) < 0)
                {
                    // Configuration failed, but don't throw - let the caller handle it
                }
            },
            [](int fd)
            {
                close(fd);
            });

        // Check if the file descriptor is valid
        if (_i2c_fd.get() < 0)
        {
            return false;
        }

        // Set slave address
        if (!_setSlaveAddress())
        {
            return false;
        }

        // Perform device detection
        if (!_performDeviceDetection())
        {
            return false;
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
        if (!isConnected())
        {
            return false;
        }

        // Try to read from register 0x00 (WHO_AM_I or similar)
        // This is a simple device detection - in a real implementation,
        // you would read a specific WHO_AM_I register and check for expected value
        struct i2c_msg msg;
        struct i2c_rdwr_ioctl_data msgset;

        uint8_t reg_addr = 0x00;

        // First message: write register address
        msg.addr = _device_address;
        msg.flags = 0;
        msg.len = 1;
        msg.buf = &reg_addr;

        msgset.msgs = &msg;
        msgset.nmsgs = 1;

        // Just try to address the device - if it ACKs, it's present
        if (ioctl(_i2c_fd.get(), I2C_RDWR, &msgset) < 0)
        {
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
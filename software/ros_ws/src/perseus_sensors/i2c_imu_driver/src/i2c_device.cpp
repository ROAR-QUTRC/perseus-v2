#include "i2c_imu_driver/i2c_device.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <vector>

namespace i2c_imu_driver
{
    namespace
    {
        /**
         * @brief Validate I2C bus path
         *
         * Ensures path points to a valid I2C device file.
         * Valid formats: /dev/i2c-N where N is 0-255
         *
         * @param path Path to validate
         * @return true if valid I2C device path, false otherwise
         */
        bool isValidI2cPath(const std::string& path)
        {
            // Must start with /dev/i2c-
            const std::string prefix = "/dev/i2c-";
            if (path.rfind(prefix, 0) != 0)
            {
                return false;
            }

            // Extract and validate bus number
            std::string bus_num = path.substr(prefix.length());

            // Must be digits only
            if (bus_num.empty() || !std::all_of(bus_num.begin(), bus_num.end(), ::isdigit))
            {
                return false;
            }

            // Bus number should be reasonable (0-255)
            try
            {
                int num = std::stoi(bus_num);
                return num >= 0 && num <= 255;
            }
            catch (...)
            {
                return false;
            }
        }
    }  // anonymous namespace

    I2cDevice::I2cDevice(const std::string& bus_path, uint8_t device_address)
        : _bus_path(bus_path),
          _device_address(device_address),
          _i2c_fd(
              [this]() -> int
              {
                  // Validate I2C bus path before attempting to open
                  if (!isValidI2cPath(_bus_path))
                  {
                      throw std::invalid_argument("Invalid I2C bus path: " + _bus_path +
                                                  ". Must be /dev/i2c-N where N is 0-255");
                  }
                  return open(_bus_path.c_str(), O_RDWR);
              },
              [this](int fd)
              {
                  if (ioctl(fd, I2C_SLAVE, _device_address) < 0)
                  {
                      throw std::runtime_error("Failed to set I2C slave address: " +
                                               std::string(strerror(errno)));
                  }
              })
    {
        // Device detection is handled by sensor-specific initialization code
        // which can check device-specific WHO_AM_I registers
    }

    bool I2cDevice::isConnected() const
    {
        return _i2c_fd.get() >= 0;
    }

    std::optional<uint8_t> I2cDevice::readRegister(uint8_t reg_address)
    {
        if (!isConnected())
        {
            return std::nullopt;
        }

        uint8_t buffer[1];
        if (!readRegisters(reg_address, buffer, 1))
        {
            return std::nullopt;
        }

        return buffer[0];
    }

    bool I2cDevice::writeRegister(uint8_t reg_address, uint8_t value)
    {
        if (!isConnected())
        {
            return false;
        }

        return writeRegisters(reg_address, &value, 1);
    }

    bool I2cDevice::readRegisters(uint8_t reg_address, uint8_t* buffer, size_t length)
    {
        // Maximum I2C transaction size (conservative limit for compatibility)
        constexpr size_t MAX_I2C_TRANSACTION_SIZE = 255;

        if (!isConnected() || buffer == nullptr || length == 0 || length > MAX_I2C_TRANSACTION_SIZE)
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

    bool I2cDevice::writeRegisters(uint8_t reg_address, const uint8_t* buffer, size_t length)
    {
        // Maximum I2C transaction size (conservative limit for compatibility)
        constexpr size_t MAX_I2C_TRANSACTION_SIZE = 255;

        if (!isConnected() || buffer == nullptr || length == 0 || length > MAX_I2C_TRANSACTION_SIZE)
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

}  // namespace i2c_imu_driver
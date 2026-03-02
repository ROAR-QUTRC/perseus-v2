#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "fd_wrapper.hpp"

namespace as7343_driver
{

    class I2cDevice
    {
    public:
        I2cDevice(const std::string& bus_path, uint8_t device_address);

        bool is_connected() const;

        std::optional<uint8_t> read_register(uint8_t reg_address);
        bool write_register(uint8_t reg_address, uint8_t value);
        bool read_registers(uint8_t reg_address, uint8_t* buffer, size_t length);
        bool write_registers(uint8_t reg_address, const uint8_t* buffer, size_t length);

        uint8_t get_device_address() const { return _device_address; }
        const std::string& get_bus_path() const { return _bus_path; }

    private:
        std::string _bus_path;
        uint8_t _device_address;
        FdWrapper _i2c_fd;
    };

}  // namespace as7343_driver

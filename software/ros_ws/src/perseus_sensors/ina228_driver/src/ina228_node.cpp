#include "ina228_driver/ina228_node.hpp"

#include "ina228_driver/ina228_registers.hpp"

#include <cmath>
#include <stdexcept>
#include <thread>

namespace ina228_driver
{

    Ina228Node::Ina228Node(const rclcpp::NodeOptions& options)
        : Node("ina228_node", options)
    {
        _initialize_parameters();
        _initialize_publishers();
        _initialize_services();

        if (!_initialize_device())
        {
            if (_required)
            {
                throw std::runtime_error("Failed to initialize required INA228 device");
            }
            else
            {
                RCLCPP_WARN(get_logger(),
                            "Failed to initialize optional INA228 device, continuing without power monitor");
                return;
            }
        }

        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / _update_rate));
        _timer = create_wall_timer(timer_period, std::bind(&Ina228Node::_timer_callback, this));

        RCLCPP_INFO(get_logger(), "INA228 power monitor initialized successfully");
    }

    Ina228Node::~Ina228Node()
    {
        if (_timer)
        {
            _timer->cancel();
        }
    }

    void Ina228Node::_initialize_parameters()
    {
        declare_parameter("i2c_bus", "/dev/i2c-7");
        declare_parameter("device_address", 0x40);
        declare_parameter("update_rate", 10.0);
        declare_parameter("shunt_resistance", 0.015);
        declare_parameter("max_current", 10.0);
        declare_parameter("required", false);

        _i2c_bus_path = get_parameter("i2c_bus").as_string();

        int device_address_int = get_parameter("device_address").as_int();
        if (device_address_int < 0 || device_address_int > 127)
        {
            throw std::invalid_argument("I2C device address must be 0-127, got: " +
                                        std::to_string(device_address_int));
        }
        _device_address = static_cast<uint8_t>(device_address_int);

        _update_rate = get_parameter("update_rate").as_double();
        _shunt_resistance = get_parameter("shunt_resistance").as_double();
        _max_current = get_parameter("max_current").as_double();
        _required = get_parameter("required").as_bool();

        if (_update_rate <= 0.0)
        {
            throw std::invalid_argument("Update rate must be positive");
        }
        if (_shunt_resistance <= 0.0)
        {
            throw std::invalid_argument("Shunt resistance must be positive");
        }
        if (_max_current <= 0.0)
        {
            throw std::invalid_argument("Max current must be positive");
        }

        // CURRENT_LSB = max_current / 2^19
        _current_lsb = _max_current / static_cast<double>(1 << 19);
    }

    void Ina228Node::_initialize_publishers()
    {
        _publisher = create_publisher<perseus_interfaces::msg::DCPowerData>("power_monitor/data", 10);
    }

    void Ina228Node::_initialize_services()
    {
        _reset_service = create_service<std_srvs::srv::Trigger>(
            "power_monitor/reset_accumulators",
            std::bind(&Ina228Node::_reset_accumulators_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
    }

    bool Ina228Node::_initialize_device()
    {
        try
        {
            _i2c_device = std::make_unique<I2cDevice>(_i2c_bus_path, _device_address);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Failed to initialize I2C device at %s:0x%02X: %s",
                         _i2c_bus_path.c_str(), _device_address, e.what());
            return false;
        }

        // Verify manufacturer ID
        uint16_t manufacturer_id = 0;
        if (!_read_register_16(REG_MANUFACTURER_ID, manufacturer_id))
        {
            RCLCPP_ERROR(get_logger(), "Failed to read MANUFACTURER_ID register");
            return false;
        }
        if (manufacturer_id != MANUFACTURER_ID)
        {
            RCLCPP_ERROR(get_logger(), "Unexpected MANUFACTURER_ID: 0x%04X (expected 0x%04X)",
                         manufacturer_id, MANUFACTURER_ID);
            return false;
        }

        // Verify device ID
        uint16_t device_id = 0;
        if (!_read_register_16(REG_DEVICE_ID, device_id))
        {
            RCLCPP_ERROR(get_logger(), "Failed to read DEVICE_ID register");
            return false;
        }
        if ((device_id & DEVICE_ID_MASK) != DEVICE_ID)
        {
            RCLCPP_ERROR(get_logger(), "Unexpected DEVICE_ID: 0x%04X (expected 0x%04X, mask 0x%04X)",
                         device_id, DEVICE_ID, DEVICE_ID_MASK);
            return false;
        }

        // Reset device to known state
        if (!_write_register_16(REG_CONFIG, CONFIG_RST))
        {
            RCLCPP_ERROR(get_logger(), "Failed to reset INA228");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Set SHUNT_CAL register
        // SHUNT_CAL = 13107.2e6 * CURRENT_LSB * R_SHUNT (for ADCRANGE = 0)
        uint16_t shunt_cal = static_cast<uint16_t>(
            std::round(SHUNT_CAL_FACTOR * _current_lsb * _shunt_resistance));
        if (!_write_register_16(REG_SHUNT_CAL, shunt_cal))
        {
            RCLCPP_ERROR(get_logger(), "Failed to write SHUNT_CAL register");
            return false;
        }

        _device_initialized = true;
        RCLCPP_INFO(get_logger(),
                    "INA228 verified at %s:0x%02X (SHUNT_CAL=%u, CURRENT_LSB=%.6e A)",
                    _i2c_bus_path.c_str(), _device_address, shunt_cal, _current_lsb);

        return true;
    }

    void Ina228Node::_timer_callback()
    {
        if (!_device_initialized || !_i2c_device || !_i2c_device->is_connected())
        {
            return;
        }

        try
        {
            perseus_interfaces::msg::DCPowerData msg;
            msg.header.stamp = now();
            msg.bus_voltage = _read_bus_voltage();
            msg.shunt_voltage = _read_shunt_voltage();
            msg.current = _read_current();
            msg.power = _read_power();
            msg.energy = _read_energy();
            msg.charge = _read_charge();
            msg.die_temperature = _read_die_temperature();

            _publisher->publish(msg);
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Error reading INA228 data: %s", e.what());
        }
    }

    // --- Register helpers ---

    bool Ina228Node::_read_register_16(uint8_t reg, uint16_t& value)
    {
        uint8_t buffer[2];
        if (!_i2c_device->read_registers(reg, buffer, 2))
        {
            return false;
        }
        value = (static_cast<uint16_t>(buffer[0]) << 8) | buffer[1];
        return true;
    }

    bool Ina228Node::_read_register_24(uint8_t reg, uint32_t& value)
    {
        uint8_t buffer[3];
        if (!_i2c_device->read_registers(reg, buffer, 3))
        {
            return false;
        }
        value = (static_cast<uint32_t>(buffer[0]) << 16) |
                (static_cast<uint32_t>(buffer[1]) << 8) |
                buffer[2];
        return true;
    }

    bool Ina228Node::_read_register_40(uint8_t reg, uint64_t& value)
    {
        uint8_t buffer[5];
        if (!_i2c_device->read_registers(reg, buffer, 5))
        {
            return false;
        }
        value = (static_cast<uint64_t>(buffer[0]) << 32) |
                (static_cast<uint64_t>(buffer[1]) << 24) |
                (static_cast<uint64_t>(buffer[2]) << 16) |
                (static_cast<uint64_t>(buffer[3]) << 8) |
                buffer[4];
        return true;
    }

    bool Ina228Node::_write_register_16(uint8_t reg, uint16_t value)
    {
        uint8_t buffer[2];
        buffer[0] = (value >> 8) & 0xFF;
        buffer[1] = value & 0xFF;
        return _i2c_device->write_registers(reg, buffer, 2);
    }

    // --- Measurement readers ---

    double Ina228Node::_read_bus_voltage()
    {
        uint32_t raw = 0;
        if (!_read_register_24(REG_VBUS, raw))
        {
            return 0.0;
        }
        // 20-bit unsigned value in bits [23:4]
        uint32_t value = raw >> 4;
        return static_cast<double>(value) * VBUS_LSB;
    }

    double Ina228Node::_read_shunt_voltage()
    {
        uint32_t raw = 0;
        if (!_read_register_24(REG_VSHUNT, raw))
        {
            return 0.0;
        }
        // 20-bit signed value in bits [23:4]
        int32_t value = static_cast<int32_t>(raw >> 4);
        if (value & (1 << 19))
        {
            value -= (1 << 20);
        }
        return static_cast<double>(value) * VSHUNT_LSB;
    }

    double Ina228Node::_read_current()
    {
        uint32_t raw = 0;
        if (!_read_register_24(REG_CURRENT, raw))
        {
            return 0.0;
        }
        // 20-bit signed value in bits [23:4]
        int32_t value = static_cast<int32_t>(raw >> 4);
        if (value & (1 << 19))
        {
            value -= (1 << 20);
        }
        return static_cast<double>(value) * _current_lsb;
    }

    double Ina228Node::_read_power()
    {
        uint32_t raw = 0;
        if (!_read_register_24(REG_POWER, raw))
        {
            return 0.0;
        }
        // Full 24-bit unsigned value
        return static_cast<double>(raw) * POWER_SCALE * _current_lsb;
    }

    double Ina228Node::_read_energy()
    {
        uint64_t raw = 0;
        if (!_read_register_40(REG_ENERGY, raw))
        {
            return 0.0;
        }
        // Full 40-bit unsigned value
        return static_cast<double>(raw) * ENERGY_SCALE * _current_lsb;
    }

    double Ina228Node::_read_charge()
    {
        uint64_t raw = 0;
        if (!_read_register_40(REG_CHARGE, raw))
        {
            return 0.0;
        }
        // 40-bit signed (two's complement)
        int64_t value = static_cast<int64_t>(raw);
        if (raw & (1ULL << 39))
        {
            value -= (1LL << 40);
        }
        return static_cast<double>(value) * _current_lsb;
    }

    double Ina228Node::_read_die_temperature()
    {
        uint16_t raw = 0;
        if (!_read_register_16(REG_DIETEMP, raw))
        {
            return 0.0;
        }
        // Full 16-bit signed value, LSB = 7.8125 m°C
        return static_cast<double>(static_cast<int16_t>(raw)) * DIETEMP_LSB;
    }

    // --- Service callbacks ---

    void Ina228Node::_reset_accumulators_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!_device_initialized || !_i2c_device || !_i2c_device->is_connected())
        {
            response->success = false;
            response->message = "INA228 device not initialized";
            return;
        }

        // Read current CONFIG value and set RSTACC bit
        uint16_t config = 0;
        if (!_read_register_16(REG_CONFIG, config))
        {
            response->success = false;
            response->message = "Failed to read CONFIG register";
            return;
        }

        if (!_write_register_16(REG_CONFIG, config | CONFIG_RSTACC))
        {
            response->success = false;
            response->message = "Failed to write CONFIG register";
            return;
        }

        response->success = true;
        response->message = "Energy and charge accumulators reset";
        RCLCPP_INFO(get_logger(), "Energy and charge accumulators reset");
    }

}  // namespace ina228_driver

#include "i2c_imu_driver/i2c_imu_hardware_interface.hpp"

#include <cmath>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace i2c_imu_driver
{

    const std::vector<std::string> I2cImuHardwareInterface::_state_interface_names = {
        "linear_acceleration.x",
        "linear_acceleration.y",
        "linear_acceleration.z",
        "angular_velocity.x",
        "angular_velocity.y",
        "angular_velocity.z",
        "temperature"};

    I2cImuHardwareInterface::I2cImuHardwareInterface()
        : _sensor_states(_state_interface_names.size(), 0.0)
    {
    }

    I2cImuHardwareInterface::~I2cImuHardwareInterface()
    {
        if (_reading_active.load())
        {
            _reading_active.store(false);
            if (_read_thread.joinable())
            {
                _read_thread.join();
            }
        }
    }

    hardware_interface::CallbackReturn I2cImuHardwareInterface::on_configure(
        const rclcpp_lifecycle::State& /* previous_state */)
    {
        // Get parameters from hardware info (no hardware claiming in on_configure)
        _i2c_bus_path = info_.hardware_parameters.at("i2c_bus");
        _device_address = static_cast<uint8_t>(std::stoi(info_.hardware_parameters.at("device_address"), nullptr, 0));
        _timeout_ms = std::chrono::milliseconds(std::stoi(info_.hardware_parameters.at("timeout_ms")));
        _retry_count = std::stoi(info_.hardware_parameters.at("retry_count"));
        _required = info_.hardware_parameters.at("required") == "true";

        // Load calibration parameters if available
        if (info_.hardware_parameters.count("accel_scale_x"))
        {
            _accel_scale_x = std::stod(info_.hardware_parameters.at("accel_scale_x"));
        }
        if (info_.hardware_parameters.count("accel_scale_y"))
        {
            _accel_scale_y = std::stod(info_.hardware_parameters.at("accel_scale_y"));
        }
        if (info_.hardware_parameters.count("accel_scale_z"))
        {
            _accel_scale_z = std::stod(info_.hardware_parameters.at("accel_scale_z"));
        }
        if (info_.hardware_parameters.count("accel_offset_x"))
        {
            _accel_offset_x = std::stod(info_.hardware_parameters.at("accel_offset_x"));
        }
        if (info_.hardware_parameters.count("accel_offset_y"))
        {
            _accel_offset_y = std::stod(info_.hardware_parameters.at("accel_offset_y"));
        }
        if (info_.hardware_parameters.count("accel_offset_z"))
        {
            _accel_offset_z = std::stod(info_.hardware_parameters.at("accel_offset_z"));
        }

        if (info_.hardware_parameters.count("gyro_scale_x"))
        {
            _gyro_scale_x = std::stod(info_.hardware_parameters.at("gyro_scale_x"));
        }
        if (info_.hardware_parameters.count("gyro_scale_y"))
        {
            _gyro_scale_y = std::stod(info_.hardware_parameters.at("gyro_scale_y"));
        }
        if (info_.hardware_parameters.count("gyro_scale_z"))
        {
            _gyro_scale_z = std::stod(info_.hardware_parameters.at("gyro_scale_z"));
        }
        if (info_.hardware_parameters.count("gyro_offset_x"))
        {
            _gyro_offset_x = std::stod(info_.hardware_parameters.at("gyro_offset_x"));
        }
        if (info_.hardware_parameters.count("gyro_offset_y"))
        {
            _gyro_offset_y = std::stod(info_.hardware_parameters.at("gyro_offset_y"));
        }
        if (info_.hardware_parameters.count("gyro_offset_z"))
        {
            _gyro_offset_z = std::stod(info_.hardware_parameters.at("gyro_offset_z"));
        }

        RCLCPP_INFO(rclcpp::get_logger("I2cImuHardwareInterface"),
                    "I2C IMU hardware interface configured with parameters from %s:0x%02X",
                    _i2c_bus_path.c_str(), _device_address);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn I2cImuHardwareInterface::on_activate(
        const rclcpp_lifecycle::State& /* previous_state */)
    {
        // Claim hardware: Create and initialize I2C device (RAII pattern)
        try
        {
            _i2c_device = std::make_unique<I2cDevice>(_i2c_bus_path, _device_address);
        }
        catch (const std::exception& e)
        {
            if (_required)
            {
                RCLCPP_ERROR(rclcpp::get_logger("I2cImuHardwareInterface"),
                             "Failed to initialize required I2C IMU device at %s:0x%02X: %s",
                             _i2c_bus_path.c_str(), _device_address, e.what());
                return hardware_interface::CallbackReturn::ERROR;
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("I2cImuHardwareInterface"),
                            "Failed to initialize optional I2C IMU device at %s:0x%02X: %s, continuing without IMU",
                            _i2c_bus_path.c_str(), _device_address, e.what());
                return hardware_interface::CallbackReturn::SUCCESS;
            }
        }

        // Configure the sensor
        if (!_initializeSensor())
        {
            _i2c_device = nullptr;  // Release hardware on failure
            if (_required)
            {
                RCLCPP_ERROR(rclcpp::get_logger("I2cImuHardwareInterface"),
                             "Failed to configure required I2C IMU sensor");
                return hardware_interface::CallbackReturn::ERROR;
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("I2cImuHardwareInterface"),
                            "Failed to configure optional I2C IMU sensor, continuing without IMU");
                return hardware_interface::CallbackReturn::SUCCESS;
            }
        }

        // Start the read thread
        _reading_active.store(true);
        _read_thread = std::thread(&I2cImuHardwareInterface::_readLoop, this);

        RCLCPP_INFO(rclcpp::get_logger("I2cImuHardwareInterface"),
                    "I2C IMU hardware interface activated at %s:0x%02X",
                    _i2c_bus_path.c_str(), _device_address);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn I2cImuHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State& /* previous_state */)
    {
        // Stop the read thread
        _reading_active.store(false);
        if (_read_thread.joinable())
        {
            _read_thread.join();
        }

        // Release hardware: Destroy I2C device (RAII cleanup)
        _i2c_device = nullptr;

        RCLCPP_INFO(rclcpp::get_logger("I2cImuHardwareInterface"),
                    "I2C IMU hardware interface deactivated");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type I2cImuHardwareInterface::read(
        const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
    {
        if (!_i2c_device || !_i2c_device->isConnected())
        {
            return hardware_interface::return_type::OK;
        }

        // Copy current sensor data to state interfaces
        {
            std::lock_guard<std::mutex> lock(_data_mutex);
            _sensor_states[0] = _current_data.accel_x;
            _sensor_states[1] = _current_data.accel_y;
            _sensor_states[2] = _current_data.accel_z;
            _sensor_states[3] = _current_data.gyro_x;
            _sensor_states[4] = _current_data.gyro_y;
            _sensor_states[5] = _current_data.gyro_z;
            _sensor_states[6] = _current_data.temperature;
        }

        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> I2cImuHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (size_t i = 0; i < _state_interface_names.size(); ++i)
        {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    info_.name, _state_interface_names[i], &_sensor_states[i]));
        }

        return state_interfaces;
    }

    void I2cImuHardwareInterface::_readLoop()
    {
        while (_reading_active.load())
        {
            if (_i2c_device && _i2c_device->isConnected())
            {
                try
                {
                    ImuData raw_data = _readRawData();
                    ImuData calibrated_data = _applyCalibration(raw_data);

                    {
                        std::lock_guard<std::mutex> lock(_data_mutex);
                        _current_data = calibrated_data;
                    }
                }
                catch (const std::exception& e)
                {
                    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("I2cImuHardwareInterface"),
                                         *rclcpp::Clock::make_shared(RCL_ROS_TIME), 1000,
                                         "Error reading IMU data: %s", e.what());
                }
            }

            // Sleep for a short period to avoid overwhelming the I2C bus
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    I2cImuHardwareInterface::ImuData I2cImuHardwareInterface::_readRawData()
    {
        ImuData data;
        data.timestamp = std::chrono::steady_clock::now();

        if (!_i2c_device || !_i2c_device->isConnected())
        {
            return data;
        }

        // This is a generic implementation - in a real scenario, you would
        // read from specific registers based on your IMU chip datasheet.
        // For example, for an MPU6050:
        // - Accelerometer: registers 0x3B-0x40
        // - Gyroscope: registers 0x43-0x48
        // - Temperature: registers 0x41-0x42

        uint8_t accel_data[6];
        uint8_t gyro_data[6];
        uint8_t temp_data[2];

        // Read accelerometer data (example registers)
        if (_i2c_device->readRegisters(0x3B, accel_data, 6, _timeout_ms))
        {
            // Convert raw data to acceleration values (example conversion)
            int16_t accel_x_raw = (accel_data[0] << 8) | accel_data[1];
            int16_t accel_y_raw = (accel_data[2] << 8) | accel_data[3];
            int16_t accel_z_raw = (accel_data[4] << 8) | accel_data[5];

            // Convert to m/s² (example scale factor for ±2g range)
            constexpr double accel_scale = 9.81 / 16384.0;
            data.accel_x = accel_x_raw * accel_scale;
            data.accel_y = accel_y_raw * accel_scale;
            data.accel_z = accel_z_raw * accel_scale;
        }

        // Read gyroscope data (example registers)
        if (_i2c_device->readRegisters(0x43, gyro_data, 6, _timeout_ms))
        {
            // Convert raw data to angular velocity values (example conversion)
            int16_t gyro_x_raw = (gyro_data[0] << 8) | gyro_data[1];
            int16_t gyro_y_raw = (gyro_data[2] << 8) | gyro_data[3];
            int16_t gyro_z_raw = (gyro_data[4] << 8) | gyro_data[5];

            // Convert to rad/s (example scale factor for ±250°/s range)
            constexpr double gyro_scale = (250.0 * M_PI / 180.0) / 32768.0;
            data.gyro_x = gyro_x_raw * gyro_scale;
            data.gyro_y = gyro_y_raw * gyro_scale;
            data.gyro_z = gyro_z_raw * gyro_scale;
        }

        // Read temperature data (example registers)
        if (_i2c_device->readRegisters(0x41, temp_data, 2, _timeout_ms))
        {
            // Convert raw data to temperature (example conversion)
            int16_t temp_raw = (temp_data[0] << 8) | temp_data[1];
            data.temperature = temp_raw / 340.0 + 36.53;  // Example MPU6050 formula
        }

        return data;
    }

    I2cImuHardwareInterface::ImuData I2cImuHardwareInterface::_applyCalibration(const ImuData& raw_data)
    {
        ImuData calibrated_data = raw_data;

        // Apply calibration to accelerometer data
        calibrated_data.accel_x = (raw_data.accel_x - _accel_offset_x) * _accel_scale_x;
        calibrated_data.accel_y = (raw_data.accel_y - _accel_offset_y) * _accel_scale_y;
        calibrated_data.accel_z = (raw_data.accel_z - _accel_offset_z) * _accel_scale_z;

        // Apply calibration to gyroscope data
        calibrated_data.gyro_x = (raw_data.gyro_x - _gyro_offset_x) * _gyro_scale_x;
        calibrated_data.gyro_y = (raw_data.gyro_y - _gyro_offset_y) * _gyro_scale_y;
        calibrated_data.gyro_z = (raw_data.gyro_z - _gyro_offset_z) * _gyro_scale_z;

        return calibrated_data;
    }

    bool I2cImuHardwareInterface::_initializeSensor()
    {
        if (!_i2c_device || !_i2c_device->isConnected())
        {
            return false;
        }

        // This is a generic initialization - in a real scenario, you would
        // configure your specific IMU based on its datasheet.
        // For example, for an MPU6050:
        // - Wake up the device (register 0x6B)
        // - Configure accelerometer range (register 0x1C)
        // - Configure gyroscope range (register 0x1B)
        // - Configure sample rate (register 0x19)

        int retry_count = 0;
        while (retry_count < _retry_count)
        {
            try
            {
                // Example: Wake up device (clear sleep bit)
                if (!_i2c_device->writeRegister(0x6B, 0x00, _timeout_ms))
                {
                    throw std::runtime_error("Failed to wake up device");
                }

                // Example: Configure accelerometer to ±2g range
                if (!_i2c_device->writeRegister(0x1C, 0x00, _timeout_ms))
                {
                    throw std::runtime_error("Failed to configure accelerometer range");
                }

                // Example: Configure gyroscope to ±250°/s range
                if (!_i2c_device->writeRegister(0x1B, 0x00, _timeout_ms))
                {
                    throw std::runtime_error("Failed to configure gyroscope range");
                }

                // Example: Set sample rate to 1kHz
                if (!_i2c_device->writeRegister(0x19, 0x07, _timeout_ms))
                {
                    throw std::runtime_error("Failed to configure sample rate");
                }

                return true;
            }
            catch (const std::exception& e)
            {
                retry_count++;
                if (retry_count < _retry_count)
                {
                    RCLCPP_WARN(rclcpp::get_logger("I2cImuHardwareInterface"),
                                "Sensor initialization failed (attempt %d/%d): %s",
                                retry_count, _retry_count, e.what());
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("I2cImuHardwareInterface"),
                                 "Sensor initialization failed after %d attempts: %s",
                                 _retry_count, e.what());
                }
            }
        }

        return false;
    }

}  // namespace i2c_imu_driver
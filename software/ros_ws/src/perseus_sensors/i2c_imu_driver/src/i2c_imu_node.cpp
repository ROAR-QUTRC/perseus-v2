#include "i2c_imu_driver/i2c_imu_node.hpp"

#include <cmath>
#include <stdexcept>
#include <thread>

namespace i2c_imu_driver
{

    I2cImuNode::I2cImuNode(const rclcpp::NodeOptions& options)
        : Node("i2c_imu_node", options),
          _device_config(nullptr)
    {
        // Initialize parameters
        _initializeParameters();

        // Initialize publishers
        _initializePublishers();

        // Initialize I2C device
        if (!_initializeDevice())
        {
            if (_required)
            {
                throw std::runtime_error("Failed to initialize required I2C IMU device");
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Failed to initialize optional I2C IMU device, continuing without IMU");
                return;
            }
        }

        // Start timer for periodic reading
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / _update_rate));
        _timer = create_wall_timer(timer_period, std::bind(&I2cImuNode::_timerCallback, this));

        RCLCPP_INFO(get_logger(), "I2C IMU node initialized successfully with device: %s", _device_type.c_str());
    }

    I2cImuNode::~I2cImuNode()
    {
        if (_timer)
        {
            _timer->cancel();
        }
    }

    void I2cImuNode::_initializeParameters()
    {
        // Declare and get parameters
        declare_parameter("i2c_bus", "/dev/i2c-7");
        declare_parameter("device_type", ImuDeviceRegistry::getDefaultDevice());
        declare_parameter("device_address", 0x6A);
        declare_parameter("update_rate", 100.0);
        declare_parameter("frame_id", "imu_link");
        declare_parameter("required", false);
        declare_parameter("timeout_ms", 1000);
        declare_parameter("retry_count", 3);

        // Calibration parameters
        declare_parameter("accel_scale_x", 1.0);
        declare_parameter("accel_scale_y", 1.0);
        declare_parameter("accel_scale_z", 1.0);
        declare_parameter("accel_offset_x", 0.0);
        declare_parameter("accel_offset_y", 0.0);
        declare_parameter("accel_offset_z", 0.0);

        declare_parameter("gyro_scale_x", 1.0);
        declare_parameter("gyro_scale_y", 1.0);
        declare_parameter("gyro_scale_z", 1.0);
        declare_parameter("gyro_offset_x", 0.0);
        declare_parameter("gyro_offset_y", 0.0);
        declare_parameter("gyro_offset_z", 0.0);

        // Covariance matrices
        declare_parameter("orientation_covariance", std::vector<double>{
                                                        0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1});
        declare_parameter("angular_velocity_covariance", std::vector<double>{
                                                             0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01});
        declare_parameter("linear_acceleration_covariance", std::vector<double>{
                                                                0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1});

        // Get parameter values
        _i2c_bus_path = get_parameter("i2c_bus").as_string();
        _device_type = get_parameter("device_type").as_string();
        _device_address = static_cast<uint8_t>(get_parameter("device_address").as_int());
        _update_rate = get_parameter("update_rate").as_double();
        _frame_id = get_parameter("frame_id").as_string();
        _required = get_parameter("required").as_bool();
        _timeout_ms = std::chrono::milliseconds(get_parameter("timeout_ms").as_int());
        _retry_count = get_parameter("retry_count").as_int();

        // Get device configuration
        _device_config = ImuDeviceRegistry::getDeviceConfig(_device_type);
        if (!_device_config)
        {
            auto supported_devices = ImuDeviceRegistry::getSupportedDevices();
            std::string supported_list;
            for (const auto& device : supported_devices)
            {
                if (!supported_list.empty())
                    supported_list += ", ";
                supported_list += device;
            }
            throw std::runtime_error("Unsupported device type: " + _device_type +
                                     ". Supported devices: " + supported_list);
        }

        // Use device default address if not explicitly set
        if (get_parameter("device_address").as_int() == 0x6A)  // Default value check
        {
            _device_address = _device_config->default_address;
        }

        // Calibration parameters
        _accel_scale_x = get_parameter("accel_scale_x").as_double();
        _accel_scale_y = get_parameter("accel_scale_y").as_double();
        _accel_scale_z = get_parameter("accel_scale_z").as_double();
        _accel_offset_x = get_parameter("accel_offset_x").as_double();
        _accel_offset_y = get_parameter("accel_offset_y").as_double();
        _accel_offset_z = get_parameter("accel_offset_z").as_double();

        _gyro_scale_x = get_parameter("gyro_scale_x").as_double();
        _gyro_scale_y = get_parameter("gyro_scale_y").as_double();
        _gyro_scale_z = get_parameter("gyro_scale_z").as_double();
        _gyro_offset_x = get_parameter("gyro_offset_x").as_double();
        _gyro_offset_y = get_parameter("gyro_offset_y").as_double();
        _gyro_offset_z = get_parameter("gyro_offset_z").as_double();

        // Covariance matrices
        auto orient_cov = get_parameter("orientation_covariance").as_double_array();
        auto angular_cov = get_parameter("angular_velocity_covariance").as_double_array();
        auto linear_cov = get_parameter("linear_acceleration_covariance").as_double_array();

        if (orient_cov.size() == 9)
        {
            std::copy(orient_cov.begin(), orient_cov.end(), _orientation_covariance.begin());
        }
        if (angular_cov.size() == 9)
        {
            std::copy(angular_cov.begin(), angular_cov.end(), _angular_velocity_covariance.begin());
        }
        if (linear_cov.size() == 9)
        {
            std::copy(linear_cov.begin(), linear_cov.end(), _linear_acceleration_covariance.begin());
        }

        // Validate parameters
        if (_update_rate <= 0.0)
        {
            throw std::invalid_argument("Update rate must be positive");
        }
        if (_timeout_ms.count() <= 0)
        {
            throw std::invalid_argument("Timeout must be positive");
        }
        if (_retry_count < 0)
        {
            throw std::invalid_argument("Retry count must be non-negative");
        }
    }

    void I2cImuNode::_initializePublishers()
    {
        _imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    }

    bool I2cImuNode::_initializeDevice()
    {
        // Create I2C device
        _i2c_device = std::make_unique<I2cDevice>(_i2c_bus_path, _device_address);

        // Initialize the device
        if (!_i2c_device->initialize())
        {
            RCLCPP_ERROR(get_logger(), "Failed to initialize I2C device at %s:0x%02X",
                         _i2c_bus_path.c_str(), _device_address);
            return false;
        }

        // Configure the sensor
        if (!_initializeSensor())
        {
            RCLCPP_ERROR(get_logger(), "Failed to configure I2C IMU sensor");
            return false;
        }

        _device_initialized = true;
        RCLCPP_INFO(get_logger(), "Successfully initialized I2C IMU device at %s:0x%02X",
                    _i2c_bus_path.c_str(), _device_address);

        return true;
    }

    void I2cImuNode::_timerCallback()
    {
        if (!_device_initialized || !_i2c_device || !_i2c_device->isConnected())
        {
            return;
        }

        try
        {
            // Read sensor data
            ImuData raw_data = _readSensorData();

            // Apply calibration
            ImuData calibrated_data = _applyCalibration(raw_data);

            // Convert to ROS message and publish
            sensor_msgs::msg::Imu imu_msg = _convertToRosMessage(calibrated_data);
            _imu_publisher->publish(imu_msg);

            _sequence_number++;
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Error reading IMU data: %s", e.what());
        }
    }

    I2cImuNode::ImuData I2cImuNode::_readSensorData()
    {
        ImuData data;
        data.timestamp = std::chrono::steady_clock::now();

        if (!_i2c_device || !_i2c_device->isConnected() || !_device_config)
        {
            return data;
        }

        uint8_t accel_data[6];
        uint8_t gyro_data[6];
        uint8_t temp_data[2];

        // Read accelerometer data using device-specific register
        if (_i2c_device->readRegisters(_device_config->accel_data_register, accel_data, 6, _timeout_ms))
        {
            int16_t accel_x_raw, accel_y_raw, accel_z_raw;

            if (_device_config->little_endian)
            {
                // Little-endian format (LSB first)
                accel_x_raw = (accel_data[1] << 8) | accel_data[0];
                accel_y_raw = (accel_data[3] << 8) | accel_data[2];
                accel_z_raw = (accel_data[5] << 8) | accel_data[4];
            }
            else
            {
                // Big-endian format (MSB first)
                accel_x_raw = (accel_data[0] << 8) | accel_data[1];
                accel_y_raw = (accel_data[2] << 8) | accel_data[3];
                accel_z_raw = (accel_data[4] << 8) | accel_data[5];
            }

            // Convert to m/s² using device-specific scale factor
            data.accel_x = accel_x_raw * _device_config->accel_scale_factor;
            data.accel_y = accel_y_raw * _device_config->accel_scale_factor;
            data.accel_z = accel_z_raw * _device_config->accel_scale_factor;
        }

        // Read gyroscope data using device-specific register
        if (_i2c_device->readRegisters(_device_config->gyro_data_register, gyro_data, 6, _timeout_ms))
        {
            int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;

            if (_device_config->little_endian)
            {
                // Little-endian format (LSB first)
                gyro_x_raw = (gyro_data[1] << 8) | gyro_data[0];
                gyro_y_raw = (gyro_data[3] << 8) | gyro_data[2];
                gyro_z_raw = (gyro_data[5] << 8) | gyro_data[4];
            }
            else
            {
                // Big-endian format (MSB first)
                gyro_x_raw = (gyro_data[0] << 8) | gyro_data[1];
                gyro_y_raw = (gyro_data[2] << 8) | gyro_data[3];
                gyro_z_raw = (gyro_data[4] << 8) | gyro_data[5];
            }

            // Convert to rad/s using device-specific scale factor
            data.gyro_x = gyro_x_raw * _device_config->gyro_scale_factor;
            data.gyro_y = gyro_y_raw * _device_config->gyro_scale_factor;
            data.gyro_z = gyro_z_raw * _device_config->gyro_scale_factor;
        }

        // Read temperature data using device-specific register
        if (_i2c_device->readRegisters(_device_config->temp_data_register, temp_data, 2, _timeout_ms))
        {
            int16_t temp_raw;

            if (_device_config->little_endian)
            {
                // Little-endian format (LSB first)
                temp_raw = (temp_data[1] << 8) | temp_data[0];
            }
            else
            {
                // Big-endian format (MSB first)
                temp_raw = (temp_data[0] << 8) | temp_data[1];
            }

            // Convert to Celsius using device-specific scale factor and offset
            data.temperature = (temp_raw * _device_config->temp_scale_factor) + _device_config->temp_offset;
        }

        return data;
    }

    I2cImuNode::ImuData I2cImuNode::_applyCalibration(const ImuData& raw_data)
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

    sensor_msgs::msg::Imu I2cImuNode::_convertToRosMessage(const ImuData& imu_data)
    {
        sensor_msgs::msg::Imu imu_msg;

        // Header
        imu_msg.header.stamp = now();
        imu_msg.header.frame_id = _frame_id;

        // Linear acceleration
        imu_msg.linear_acceleration.x = imu_data.accel_x;
        imu_msg.linear_acceleration.y = imu_data.accel_y;
        imu_msg.linear_acceleration.z = imu_data.accel_z;

        // Angular velocity
        imu_msg.angular_velocity.x = imu_data.gyro_x;
        imu_msg.angular_velocity.y = imu_data.gyro_y;
        imu_msg.angular_velocity.z = imu_data.gyro_z;

        // Orientation (not provided by most IMUs, set to unknown)
        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
        imu_msg.orientation.w = 1.0;

        // Set covariance matrices
        std::copy(_orientation_covariance.begin(), _orientation_covariance.end(),
                  imu_msg.orientation_covariance.begin());
        std::copy(_angular_velocity_covariance.begin(), _angular_velocity_covariance.end(),
                  imu_msg.angular_velocity_covariance.begin());
        std::copy(_linear_acceleration_covariance.begin(), _linear_acceleration_covariance.end(),
                  imu_msg.linear_acceleration_covariance.begin());

        // Set orientation covariance to -1 to indicate unknown orientation
        imu_msg.orientation_covariance[0] = -1.0;

        return imu_msg;
    }

    bool I2cImuNode::_initializeSensor()
    {
        if (!_i2c_device || !_i2c_device->isConnected())
        {
            return false;
        }

        if (!_device_config)
        {
            RCLCPP_ERROR(get_logger(), "Device configuration not available");
            return false;
        }

        // Device-agnostic initialization sequence
        int retry_count = 0;
        while (retry_count < _retry_count)
        {
            try
            {
                // Check WHO_AM_I register
                uint8_t who_am_i;
                if (!_i2c_device->readRegisters(_device_config->who_am_i_register, &who_am_i, 1, _timeout_ms))
                {
                    throw std::runtime_error("Failed to read WHO_AM_I register");
                }
                if (who_am_i != _device_config->who_am_i_value)
                {
                    throw std::runtime_error("Invalid WHO_AM_I value: expected 0x" +
                                             std::to_string(_device_config->who_am_i_value) +
                                             ", got 0x" + std::to_string(who_am_i));
                }

                // Reset device
                if (!_i2c_device->writeRegister(_device_config->reset_register, _device_config->reset_value, _timeout_ms))
                {
                    throw std::runtime_error("Failed to reset device");
                }

                // Wait for reset to complete
                std::this_thread::sleep_for(std::chrono::milliseconds(50));

                // Configure accelerometer
                if (!_i2c_device->writeRegister(_device_config->accel_config_register, _device_config->accel_config_value, _timeout_ms))
                {
                    throw std::runtime_error("Failed to configure accelerometer");
                }

                // Configure gyroscope
                if (!_i2c_device->writeRegister(_device_config->gyro_config_register, _device_config->gyro_config_value, _timeout_ms))
                {
                    throw std::runtime_error("Failed to configure gyroscope");
                }

                // Configure control register
                if (!_i2c_device->writeRegister(_device_config->ctrl_register, _device_config->ctrl_value, _timeout_ms))
                {
                    throw std::runtime_error("Failed to configure control register");
                }

                return true;
            }
            catch (const std::exception& e)
            {
                retry_count++;
                if (retry_count < _retry_count)
                {
                    RCLCPP_WARN(get_logger(), "Sensor initialization failed (attempt %d/%d): %s",
                                retry_count, _retry_count, e.what());
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                else
                {
                    RCLCPP_ERROR(get_logger(), "Sensor initialization failed after %d attempts: %s",
                                 _retry_count, e.what());
                }
            }
        }

        return false;
    }

}  // namespace i2c_imu_driver
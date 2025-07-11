#include "i2c_imu_driver/imu_device_config.hpp"

#include <cmath>
#include <vector>

namespace i2c_imu_driver
{

    std::unordered_map<std::string, ImuDeviceConfig> ImuDeviceRegistry::_device_configs;
    bool ImuDeviceRegistry::_initialized = false;

    void ImuDeviceRegistry::_initializeDeviceConfigs()
    {
        if (_initialized)
            return;

        // LSM6DSOX Configuration (current default)
        _device_configs["lsm6dsox"] = ImuDeviceConfig{
            .name = "lsm6dsox",
            .default_address = 0x6A,
            .who_am_i_register = 0x0F,
            .who_am_i_value = 0x6C,
            .accel_data_register = 0x28,
            .gyro_data_register = 0x22,
            .temp_data_register = 0x20,
            .reset_register = 0x12,
            .accel_config_register = 0x10,
            .gyro_config_register = 0x11,
            .ctrl_register = 0x12,
            .reset_value = 0x01,
            .accel_config_value = 0x40,                   // ODR = 104 Hz, FS = ±2g
            .gyro_config_value = 0x40,                    // ODR = 104 Hz, FS = ±250 dps
            .ctrl_value = 0x40,                           // Enable block data update
            .accel_scale_factor = 0.061e-3 * 9.81,        // ±2g range: 0.061 mg/LSB to m/s²
            .gyro_scale_factor = 8.75e-3 * M_PI / 180.0,  // ±250 dps range: 8.75 mdps/LSB to rad/s
            .temp_scale_factor = 1.0 / 256.0,             // 256 LSB/°C
            .temp_offset = 25.0,                          // 25°C offset
            .little_endian = true,
            .data_bytes_per_axis = 2};

        // MPU6050 Configuration
        _device_configs["mpu6050"] = ImuDeviceConfig{
            .name = "mpu6050",
            .default_address = 0x68,
            .who_am_i_register = 0x75,
            .who_am_i_value = 0x68,
            .accel_data_register = 0x3B,
            .gyro_data_register = 0x43,
            .temp_data_register = 0x41,
            .reset_register = 0x6B,
            .accel_config_register = 0x1C,
            .gyro_config_register = 0x1B,
            .ctrl_register = 0x19,
            .reset_value = 0x00,                                    // Exit sleep mode
            .accel_config_value = 0x00,                             // ±2g range
            .gyro_config_value = 0x00,                              // ±250 dps range
            .ctrl_value = 0x07,                                     // Sample rate divider (1kHz / (1 + 7) = 125 Hz)
            .accel_scale_factor = 9.81 / 16384.0,                   // ±2g range: 16384 LSB/g to m/s²
            .gyro_scale_factor = (250.0 * M_PI / 180.0) / 32768.0,  // ±250 dps range to rad/s
            .temp_scale_factor = 1.0 / 340.0,                       // 340 LSB/°C
            .temp_offset = 36.53,                                   // 36.53°C offset
            .little_endian = false,                                 // Big-endian format
            .data_bytes_per_axis = 2};

        // MPU9250 Configuration (similar to MPU6050 but with different WHO_AM_I)
        _device_configs["mpu9250"] = ImuDeviceConfig{
            .name = "mpu9250",
            .default_address = 0x68,
            .who_am_i_register = 0x75,
            .who_am_i_value = 0x71,
            .accel_data_register = 0x3B,
            .gyro_data_register = 0x43,
            .temp_data_register = 0x41,
            .reset_register = 0x6B,
            .accel_config_register = 0x1C,
            .gyro_config_register = 0x1B,
            .ctrl_register = 0x19,
            .reset_value = 0x00,                                    // Exit sleep mode
            .accel_config_value = 0x00,                             // ±2g range
            .gyro_config_value = 0x00,                              // ±250 dps range
            .ctrl_value = 0x07,                                     // Sample rate divider (1kHz / (1 + 7) = 125 Hz)
            .accel_scale_factor = 9.81 / 16384.0,                   // ±2g range: 16384 LSB/g to m/s²
            .gyro_scale_factor = (250.0 * M_PI / 180.0) / 32768.0,  // ±250 dps range to rad/s
            .temp_scale_factor = 1.0 / 340.0,                       // 340 LSB/°C
            .temp_offset = 36.53,                                   // 36.53°C offset
            .little_endian = false,                                 // Big-endian format
            .data_bytes_per_axis = 2};

        // LSM6DS3 Configuration
        _device_configs["lsm6ds3"] = ImuDeviceConfig{
            .name = "lsm6ds3",
            .default_address = 0x6A,
            .who_am_i_register = 0x0F,
            .who_am_i_value = 0x69,
            .accel_data_register = 0x28,
            .gyro_data_register = 0x22,
            .temp_data_register = 0x20,
            .reset_register = 0x12,
            .accel_config_register = 0x10,
            .gyro_config_register = 0x11,
            .ctrl_register = 0x12,
            .reset_value = 0x01,
            .accel_config_value = 0x40,                   // ODR = 104 Hz, FS = ±2g
            .gyro_config_value = 0x40,                    // ODR = 104 Hz, FS = ±250 dps
            .ctrl_value = 0x40,                           // Enable block data update
            .accel_scale_factor = 0.061e-3 * 9.81,        // ±2g range: 0.061 mg/LSB to m/s²
            .gyro_scale_factor = 8.75e-3 * M_PI / 180.0,  // ±250 dps range: 8.75 mdps/LSB to rad/s
            .temp_scale_factor = 1.0 / 256.0,             // 256 LSB/°C
            .temp_offset = 25.0,                          // 25°C offset
            .little_endian = true,
            .data_bytes_per_axis = 2};

        _initialized = true;
    }

    const ImuDeviceConfig* ImuDeviceRegistry::getDeviceConfig(const std::string& device_name)
    {
        _initializeDeviceConfigs();

        auto it = _device_configs.find(device_name);
        if (it != _device_configs.end())
        {
            return &it->second;
        }
        return nullptr;
    }

    std::vector<std::string> ImuDeviceRegistry::getSupportedDevices()
    {
        _initializeDeviceConfigs();

        std::vector<std::string> devices;
        for (const auto& pair : _device_configs)
        {
            devices.push_back(pair.first);
        }
        return devices;
    }

    std::string ImuDeviceRegistry::getDefaultDevice()
    {
        return "lsm6dsox";
    }

}  // namespace i2c_imu_driver
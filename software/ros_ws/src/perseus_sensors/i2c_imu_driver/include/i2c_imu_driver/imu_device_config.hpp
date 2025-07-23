#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace i2c_imu_driver
{

    /**
     * @brief Configuration structure for different IMU devices
     */
    struct ImuDeviceConfig
    {
        // Device identification
        std::string name;
        uint8_t default_address;
        uint8_t who_am_i_register;
        uint8_t who_am_i_value;

        // Data registers
        uint8_t accel_data_register;
        uint8_t gyro_data_register;
        uint8_t temp_data_register;

        // Control registers
        uint8_t reset_register;
        uint8_t accel_config_register;
        uint8_t gyro_config_register;
        uint8_t ctrl_register;

        // Configuration values
        uint8_t reset_value;
        uint8_t accel_config_value;
        uint8_t gyro_config_value;
        uint8_t ctrl_value;

        // Scale factors (for converting raw data to physical units)
        double accel_scale_factor;  // to convert to m/s²
        double gyro_scale_factor;   // to convert to rad/s
        double temp_scale_factor;   // to convert to °C
        double temp_offset;         // temperature offset in °C

        // Data format
        bool little_endian;
        uint8_t data_bytes_per_axis;
    };

    /**
     * @brief Registry of supported IMU devices
     * @details This class provides a centralized registry for different IMU device configurations.
     *          It maintains device-specific settings including register addresses, scale factors,
     *          and configuration values for various IMU sensors (e.g., MPU6050, ICM20948).
     *          The registry follows a singleton pattern and lazy initialization to ensure
     *          device configurations are loaded only when needed.
     * 
     * Example usage:
     * @code
     * const auto* config = ImuDeviceRegistry::getDeviceConfig("MPU6050");
     * if (config) {
     *     // Use configuration for device initialization
     *     uint8_t who_am_i = config->who_am_i_value;
     * }
     * @endcode
     */
    class ImuDeviceRegistry
    {
    public:
        /**
         * @brief Get device configuration by name
         * @param device_name Name of the device
         * @return Device configuration, or nullptr if not found
         */
        static const ImuDeviceConfig* getDeviceConfig(const std::string& device_name);

        /**
         * @brief Get list of all supported device names
         * @return Vector of device names
         */
        static std::vector<std::string> getSupportedDevices();

        /**
         * @brief Get default device name
         * @return Default device name
         */
        static std::string getDefaultDevice();

    private:
        /**
         * @brief Initialize device configurations
         */
        static void _initializeDeviceConfigs();

        static std::unordered_map<std::string, ImuDeviceConfig> _device_configs;
        static bool _initialized;
    };

}  // namespace i2c_imu_driver
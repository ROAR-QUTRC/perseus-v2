#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "i2c_imu_driver/i2c_device.hpp"

namespace i2c_imu_driver
{

/**
 * @brief Hardware interface for I2C IMU sensor
 * 
 * This class extends the ROS2 hardware_interface::SensorInterface to provide
 * a standardized interface for I2C IMU sensors. It manages the device lifecycle,
 * implements async I/O with timeout protection, and provides sensor state interfaces.
 */
class I2cImuHardwareInterface : public hardware_interface::SensorInterface
{
public:
    /**
     * @brief Construct a new I2cImuHardwareInterface object
     */
    I2cImuHardwareInterface();

    /**
     * @brief Destroy the I2cImuHardwareInterface object
     */
    ~I2cImuHardwareInterface();

    /**
     * @brief Configure the hardware interface
     * 
     * @param previous_state Previous lifecycle state
     * @return CallbackReturn Configuration result
     */
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief Activate the hardware interface
     * 
     * @param previous_state Previous lifecycle state
     * @return CallbackReturn Activation result
     */
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief Deactivate the hardware interface
     * 
     * @param previous_state Previous lifecycle state
     * @return CallbackReturn Deactivation result
     */
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief Read sensor data
     * 
     * @param time Current time
     * @param period Time period since last read
     * @return return_type Read result
     */
    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    /**
     * @brief Export sensor state interfaces
     * 
     * @return std::vector<hardware_interface::StateInterface> State interfaces
     */
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

private:
    /**
     * @brief IMU sensor data structure
     */
    struct ImuData
    {
        double accel_x{0.0};          // m/s²
        double accel_y{0.0};          // m/s²
        double accel_z{0.0};          // m/s²
        double gyro_x{0.0};           // rad/s
        double gyro_y{0.0};           // rad/s
        double gyro_z{0.0};           // rad/s
        double temperature{0.0};      // °C
        std::chrono::steady_clock::time_point timestamp;
    };

    /**
     * @brief Read thread function for continuous sensor reading
     */
    void _readLoop();

    /**
     * @brief Read raw sensor data from I2C device
     * 
     * @return ImuData Raw sensor data
     */
    ImuData _readRawData();

    /**
     * @brief Apply calibration to raw sensor data
     * 
     * @param raw_data Raw sensor data
     * @return ImuData Calibrated sensor data
     */
    ImuData _applyCalibration(const ImuData& raw_data);

    /**
     * @brief Initialize IMU sensor configuration
     * 
     * @return true if initialization succeeded, false otherwise
     */
    bool _initializeSensor();

    // Configuration parameters
    std::string _i2c_bus_path;
    uint8_t _device_address;
    std::chrono::milliseconds _timeout_ms;
    int _retry_count;
    bool _required;

    // IMU calibration parameters
    double _accel_scale_x{1.0};
    double _accel_scale_y{1.0};
    double _accel_scale_z{1.0};
    double _accel_offset_x{0.0};
    double _accel_offset_y{0.0};
    double _accel_offset_z{0.0};
    
    double _gyro_scale_x{1.0};
    double _gyro_scale_y{1.0};
    double _gyro_scale_z{1.0};
    double _gyro_offset_x{0.0};
    double _gyro_offset_y{0.0};
    double _gyro_offset_z{0.0};

    // Hardware interface
    std::unique_ptr<I2cDevice> _i2c_device;

    // Thread management
    std::thread _read_thread;
    std::atomic<bool> _reading_active{false};
    std::mutex _data_mutex;

    // Sensor state data
    ImuData _current_data;
    std::vector<double> _sensor_states;

    // State interface names
    static const std::vector<std::string> _state_interface_names;
};

} // namespace i2c_imu_driver
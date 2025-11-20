#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>

#include "i2c_imu_driver/i2c_device.hpp"
#include "i2c_imu_driver/imu_data_types.hpp"
#include "i2c_imu_driver/imu_device_config.hpp"

namespace i2c_imu_driver
{

    /**
     * @brief ROS2 node for I2C IMU sensor
     *
     * This node provides a standalone ROS2 interface for I2C IMU sensors.
     * It publishes sensor_msgs/Imu messages with configurable update rate,
     * handles graceful failure when IMU is not available, and provides
     * parameter-based configuration.
     */
    class I2cImuNode : public rclcpp::Node
    {
    public:
        /**
         * @brief Construct a new I2cImuNode object
         *
         * @param options Node options for ROS2 configuration
         */
        explicit I2cImuNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        /**
         * @brief Destroy the I2cImuNode object
         */
        ~I2cImuNode();

    private:
        /**
         * @brief Initialize node parameters
         */
        void _initializeParameters();

        /**
         * @brief Initialize ROS2 publishers
         */
        void _initializePublishers();

        /**
         * @brief Initialize I2C device
         *
         * @return true if initialization succeeded, false otherwise
         */
        bool _initializeDevice();

        /**
         * @brief Timer callback for reading and publishing sensor data
         */
        void _timerCallback();

        /**
         * @brief Read raw sensor data from I2C device
         *
         * @return ImuData Raw sensor data
         */
        ImuData _readSensorData();

        /**
         * @brief Convert IMU data to ROS message
         *
         * @param imu_data Calibrated IMU data
         * @return sensor_msgs::msg::Imu ROS IMU message
         */
        sensor_msgs::msg::Imu _convertToRosMessage(const ImuData& imu_data);

        /**
         * @brief Initialize IMU sensor configuration
         *
         * @return true if initialization succeeded, false otherwise
         */
        bool _initializeSensor();

        // Node parameters
        std::string _i2c_bus_path;
        uint8_t _device_address;
        double _update_rate;
        std::string _frame_id;
        bool _required;
        int _retry_count;

        // Device configuration
        std::string _device_type;
        const ImuDeviceConfig* _device_config;

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

        // Covariance matrices (orientation not estimated by IMU)
        std::array<double, 9> _angular_velocity_covariance;
        std::array<double, 9> _linear_acceleration_covariance;

        // ROS2 components
        std::unique_ptr<I2cDevice> _i2c_device;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_publisher;
        rclcpp::TimerBase::SharedPtr _timer;

        // Status tracking
        std::atomic<bool> _device_initialized{false};
        size_t _sequence_number{0};
    };

}  // namespace i2c_imu_driver
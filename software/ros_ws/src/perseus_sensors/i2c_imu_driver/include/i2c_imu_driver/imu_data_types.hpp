#pragma once

#include <chrono>

namespace i2c_imu_driver
{

    /**
     * @brief Raw IMU sensor data
     *
     * Contains readings from accelerometer, gyroscope, and temperature sensor.
     * All values are in SI units after device-specific scale factor conversion
     * but before calibration is applied.
     */
    struct ImuData
    {
        /// Timestamp when data was captured
        std::chrono::steady_clock::time_point timestamp;

        // Linear acceleration (m/s²)
        double accel_x{0.0};
        double accel_y{0.0};
        double accel_z{0.0};

        // Angular velocity (rad/s)
        double gyro_x{0.0};
        double gyro_y{0.0};
        double gyro_z{0.0};

        // Temperature (°C)
        double temperature{0.0};
    };

    /**
     * @brief Apply calibration to IMU data
     *
     * Applies scale factors and offsets to raw sensor data. The calibration
     * formula is: calibrated = (raw - offset) * scale
     *
     * @param raw_data Input data from sensor (after scale factor conversion)
     * @param accel_scale_x Scale factor for X-axis acceleration
     * @param accel_scale_y Scale factor for Y-axis acceleration
     * @param accel_scale_z Scale factor for Z-axis acceleration
     * @param accel_offset_x Offset correction for X-axis acceleration (m/s²)
     * @param accel_offset_y Offset correction for Y-axis acceleration (m/s²)
     * @param accel_offset_z Offset correction for Z-axis acceleration (m/s²)
     * @param gyro_scale_x Scale factor for X-axis angular velocity
     * @param gyro_scale_y Scale factor for Y-axis angular velocity
     * @param gyro_scale_z Scale factor for Z-axis angular velocity
     * @param gyro_offset_x Offset correction for X-axis angular velocity (rad/s)
     * @param gyro_offset_y Offset correction for Y-axis angular velocity (rad/s)
     * @param gyro_offset_z Offset correction for Z-axis angular velocity (rad/s)
     * @return Calibrated IMU data
     */
    inline ImuData apply_calibration(
        const ImuData& raw_data,
        double accel_scale_x, double accel_scale_y, double accel_scale_z,
        double accel_offset_x, double accel_offset_y, double accel_offset_z,
        double gyro_scale_x, double gyro_scale_y, double gyro_scale_z,
        double gyro_offset_x, double gyro_offset_y, double gyro_offset_z)
    {
        ImuData calibrated_data = raw_data;

        // Apply calibration to accelerometer data: calibrated = (raw - offset) * scale
        calibrated_data.accel_x = (raw_data.accel_x - accel_offset_x) * accel_scale_x;
        calibrated_data.accel_y = (raw_data.accel_y - accel_offset_y) * accel_scale_y;
        calibrated_data.accel_z = (raw_data.accel_z - accel_offset_z) * accel_scale_z;

        // Apply calibration to gyroscope data: calibrated = (raw - offset) * scale
        calibrated_data.gyro_x = (raw_data.gyro_x - gyro_offset_x) * gyro_scale_x;
        calibrated_data.gyro_y = (raw_data.gyro_y - gyro_offset_y) * gyro_scale_y;
        calibrated_data.gyro_z = (raw_data.gyro_z - gyro_offset_z) * gyro_scale_z;

        return calibrated_data;
    }

}  // namespace i2c_imu_driver

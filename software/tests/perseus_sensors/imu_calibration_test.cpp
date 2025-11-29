/**
 * @file imu_calibration_test.cpp
 * @brief Unit tests for IMU calibration functions
 *
 * Tests the apply_calibration() function to ensure correct application
 * of scale factors and offsets to IMU sensor data.
 */

#include <gtest/gtest.h>
#include <cmath>
#include "i2c_imu_driver/imu_data_types.hpp"

using namespace i2c_imu_driver;

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Create a sample ImuData struct with known values
 */
ImuData create_test_imu_data(
    double accel_x = 1.0, double accel_y = 2.0, double accel_z = 9.81,
    double gyro_x = 0.1, double gyro_y = 0.2, double gyro_z = 0.3,
    double temp = 25.0)
{
    ImuData data;
    data.timestamp = std::chrono::steady_clock::now();
    data.accel_x = accel_x;
    data.accel_y = accel_y;
    data.accel_z = accel_z;
    data.gyro_x = gyro_x;
    data.gyro_y = gyro_y;
    data.gyro_z = gyro_z;
    data.temperature = temp;
    return data;
}

// ============================================================================
// Identity Calibration Tests
// ============================================================================

TEST(ImuCalibrationTest, IdentityCalibrationPreservesAccelData)
{
    ImuData raw = create_test_imu_data();

    // Identity calibration: scale=1, offset=0
    ImuData calibrated = apply_calibration(
        raw,
        1.0, 1.0, 1.0,  // accel scales
        0.0, 0.0, 0.0,  // accel offsets
        1.0, 1.0, 1.0,  // gyro scales
        0.0, 0.0, 0.0); // gyro offsets

    EXPECT_DOUBLE_EQ(calibrated.accel_x, raw.accel_x);
    EXPECT_DOUBLE_EQ(calibrated.accel_y, raw.accel_y);
    EXPECT_DOUBLE_EQ(calibrated.accel_z, raw.accel_z);
}

TEST(ImuCalibrationTest, IdentityCalibrationPreservesGyroData)
{
    ImuData raw = create_test_imu_data();

    ImuData calibrated = apply_calibration(
        raw,
        1.0, 1.0, 1.0,
        0.0, 0.0, 0.0,
        1.0, 1.0, 1.0,
        0.0, 0.0, 0.0);

    EXPECT_DOUBLE_EQ(calibrated.gyro_x, raw.gyro_x);
    EXPECT_DOUBLE_EQ(calibrated.gyro_y, raw.gyro_y);
    EXPECT_DOUBLE_EQ(calibrated.gyro_z, raw.gyro_z);
}

// ============================================================================
// Offset Correction Tests
// ============================================================================

TEST(ImuCalibrationTest, AccelOffsetIsSubtracted)
{
    ImuData raw = create_test_imu_data(10.0, 20.0, 30.0);

    // Scale=1, various offsets
    ImuData calibrated = apply_calibration(
        raw,
        1.0, 1.0, 1.0,
        1.0, 2.0, 3.0,  // offsets
        1.0, 1.0, 1.0,
        0.0, 0.0, 0.0);

    // calibrated = (raw - offset) * scale = raw - offset when scale=1
    EXPECT_DOUBLE_EQ(calibrated.accel_x, 9.0);   // 10 - 1
    EXPECT_DOUBLE_EQ(calibrated.accel_y, 18.0);  // 20 - 2
    EXPECT_DOUBLE_EQ(calibrated.accel_z, 27.0);  // 30 - 3
}

TEST(ImuCalibrationTest, GyroOffsetIsSubtracted)
{
    ImuData raw = create_test_imu_data(0, 0, 0, 1.0, 2.0, 3.0);

    ImuData calibrated = apply_calibration(
        raw,
        1.0, 1.0, 1.0,
        0.0, 0.0, 0.0,
        1.0, 1.0, 1.0,
        0.1, 0.2, 0.3);  // gyro offsets

    EXPECT_DOUBLE_EQ(calibrated.gyro_x, 0.9);   // 1.0 - 0.1
    EXPECT_DOUBLE_EQ(calibrated.gyro_y, 1.8);   // 2.0 - 0.2
    EXPECT_DOUBLE_EQ(calibrated.gyro_z, 2.7);   // 3.0 - 0.3
}

TEST(ImuCalibrationTest, NegativeOffsetIncreasesValue)
{
    ImuData raw = create_test_imu_data(5.0, 5.0, 5.0);

    ImuData calibrated = apply_calibration(
        raw,
        1.0, 1.0, 1.0,
        -1.0, -2.0, -3.0,  // negative offsets
        1.0, 1.0, 1.0,
        0.0, 0.0, 0.0);

    // (5 - (-1)) * 1 = 6
    EXPECT_DOUBLE_EQ(calibrated.accel_x, 6.0);
    EXPECT_DOUBLE_EQ(calibrated.accel_y, 7.0);
    EXPECT_DOUBLE_EQ(calibrated.accel_z, 8.0);
}

// ============================================================================
// Scale Factor Tests
// ============================================================================

TEST(ImuCalibrationTest, AccelScaleIsApplied)
{
    ImuData raw = create_test_imu_data(10.0, 10.0, 10.0);

    // No offsets, various scales
    ImuData calibrated = apply_calibration(
        raw,
        0.5, 2.0, 1.5,  // scales
        0.0, 0.0, 0.0,
        1.0, 1.0, 1.0,
        0.0, 0.0, 0.0);

    EXPECT_DOUBLE_EQ(calibrated.accel_x, 5.0);   // 10 * 0.5
    EXPECT_DOUBLE_EQ(calibrated.accel_y, 20.0);  // 10 * 2.0
    EXPECT_DOUBLE_EQ(calibrated.accel_z, 15.0);  // 10 * 1.5
}

TEST(ImuCalibrationTest, GyroScaleIsApplied)
{
    ImuData raw = create_test_imu_data(0, 0, 0, 1.0, 1.0, 1.0);

    ImuData calibrated = apply_calibration(
        raw,
        1.0, 1.0, 1.0,
        0.0, 0.0, 0.0,
        2.0, 3.0, 4.0,  // gyro scales
        0.0, 0.0, 0.0);

    EXPECT_DOUBLE_EQ(calibrated.gyro_x, 2.0);
    EXPECT_DOUBLE_EQ(calibrated.gyro_y, 3.0);
    EXPECT_DOUBLE_EQ(calibrated.gyro_z, 4.0);
}

// ============================================================================
// Combined Scale and Offset Tests
// ============================================================================

TEST(ImuCalibrationTest, OffsetThenScaleOrder)
{
    // Formula: calibrated = (raw - offset) * scale
    ImuData raw = create_test_imu_data(10.0, 10.0, 10.0);

    ImuData calibrated = apply_calibration(
        raw,
        2.0, 2.0, 2.0,  // scales
        3.0, 3.0, 3.0,  // offsets
        1.0, 1.0, 1.0,
        0.0, 0.0, 0.0);

    // (10 - 3) * 2 = 14
    EXPECT_DOUBLE_EQ(calibrated.accel_x, 14.0);
    EXPECT_DOUBLE_EQ(calibrated.accel_y, 14.0);
    EXPECT_DOUBLE_EQ(calibrated.accel_z, 14.0);
}

TEST(ImuCalibrationTest, FullCalibrationComputation)
{
    ImuData raw;
    raw.accel_x = 9.81;
    raw.accel_y = 0.5;
    raw.accel_z = 9.0;
    raw.gyro_x = 0.01;
    raw.gyro_y = -0.02;
    raw.gyro_z = 0.005;

    // Realistic calibration values
    ImuData calibrated = apply_calibration(
        raw,
        1.01, 0.99, 1.02,     // accel scales (slight correction)
        0.1, -0.05, 0.2,       // accel offsets (bias correction)
        1.005, 0.995, 1.0,     // gyro scales
        0.001, 0.002, -0.001); // gyro offsets (drift correction)

    // Expected: (raw - offset) * scale
    double expected_accel_x = (9.81 - 0.1) * 1.01;
    double expected_accel_y = (0.5 - (-0.05)) * 0.99;
    double expected_accel_z = (9.0 - 0.2) * 1.02;
    double expected_gyro_x = (0.01 - 0.001) * 1.005;
    double expected_gyro_y = (-0.02 - 0.002) * 0.995;
    double expected_gyro_z = (0.005 - (-0.001)) * 1.0;

    EXPECT_NEAR(calibrated.accel_x, expected_accel_x, 1e-10);
    EXPECT_NEAR(calibrated.accel_y, expected_accel_y, 1e-10);
    EXPECT_NEAR(calibrated.accel_z, expected_accel_z, 1e-10);
    EXPECT_NEAR(calibrated.gyro_x, expected_gyro_x, 1e-10);
    EXPECT_NEAR(calibrated.gyro_y, expected_gyro_y, 1e-10);
    EXPECT_NEAR(calibrated.gyro_z, expected_gyro_z, 1e-10);
}

// ============================================================================
// Data Preservation Tests
// ============================================================================

TEST(ImuCalibrationTest, TemperatureIsPreserved)
{
    ImuData raw = create_test_imu_data();
    raw.temperature = 42.5;

    ImuData calibrated = apply_calibration(
        raw,
        2.0, 2.0, 2.0,
        1.0, 1.0, 1.0,
        2.0, 2.0, 2.0,
        1.0, 1.0, 1.0);

    // Temperature should be unchanged
    EXPECT_DOUBLE_EQ(calibrated.temperature, 42.5);
}

TEST(ImuCalibrationTest, TimestampIsPreserved)
{
    ImuData raw = create_test_imu_data();
    auto original_timestamp = raw.timestamp;

    ImuData calibrated = apply_calibration(
        raw,
        2.0, 2.0, 2.0,
        1.0, 1.0, 1.0,
        2.0, 2.0, 2.0,
        1.0, 1.0, 1.0);

    EXPECT_EQ(calibrated.timestamp, original_timestamp);
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST(ImuCalibrationTest, ZeroInputData)
{
    ImuData raw = create_test_imu_data(0, 0, 0, 0, 0, 0, 0);

    ImuData calibrated = apply_calibration(
        raw,
        1.5, 1.5, 1.5,
        0.5, 0.5, 0.5,  // offsets will make result negative
        1.5, 1.5, 1.5,
        0.5, 0.5, 0.5);

    // (0 - 0.5) * 1.5 = -0.75
    EXPECT_DOUBLE_EQ(calibrated.accel_x, -0.75);
    EXPECT_DOUBLE_EQ(calibrated.accel_y, -0.75);
    EXPECT_DOUBLE_EQ(calibrated.accel_z, -0.75);
    EXPECT_DOUBLE_EQ(calibrated.gyro_x, -0.75);
    EXPECT_DOUBLE_EQ(calibrated.gyro_y, -0.75);
    EXPECT_DOUBLE_EQ(calibrated.gyro_z, -0.75);
}

TEST(ImuCalibrationTest, ZeroScaleZerosOutput)
{
    ImuData raw = create_test_imu_data(100.0, 200.0, 300.0);

    ImuData calibrated = apply_calibration(
        raw,
        0.0, 0.0, 0.0,  // zero scales
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0);

    EXPECT_DOUBLE_EQ(calibrated.accel_x, 0.0);
    EXPECT_DOUBLE_EQ(calibrated.accel_y, 0.0);
    EXPECT_DOUBLE_EQ(calibrated.accel_z, 0.0);
}

TEST(ImuCalibrationTest, NegativeScaleInvertsData)
{
    ImuData raw = create_test_imu_data(10.0, 10.0, 10.0);

    ImuData calibrated = apply_calibration(
        raw,
        -1.0, -1.0, -1.0,  // negative scales (axis inversion)
        0.0, 0.0, 0.0,
        1.0, 1.0, 1.0,
        0.0, 0.0, 0.0);

    EXPECT_DOUBLE_EQ(calibrated.accel_x, -10.0);
    EXPECT_DOUBLE_EQ(calibrated.accel_y, -10.0);
    EXPECT_DOUBLE_EQ(calibrated.accel_z, -10.0);
}

TEST(ImuCalibrationTest, LargeValuesHandledCorrectly)
{
    ImuData raw = create_test_imu_data(1e6, 1e6, 1e6, 1e4, 1e4, 1e4);

    ImuData calibrated = apply_calibration(
        raw,
        1e-6, 1e-6, 1e-6,  // very small scales
        0.0, 0.0, 0.0,
        1e-4, 1e-4, 1e-4,
        0.0, 0.0, 0.0);

    EXPECT_NEAR(calibrated.accel_x, 1.0, 1e-10);
    EXPECT_NEAR(calibrated.gyro_x, 1.0, 1e-10);
}

TEST(ImuCalibrationTest, NegativeInputData)
{
    ImuData raw = create_test_imu_data(-5.0, -10.0, -15.0, -0.5, -1.0, -1.5);

    ImuData calibrated = apply_calibration(
        raw,
        2.0, 2.0, 2.0,
        1.0, 1.0, 1.0,
        2.0, 2.0, 2.0,
        0.1, 0.1, 0.1);

    // (-5 - 1) * 2 = -12
    EXPECT_DOUBLE_EQ(calibrated.accel_x, -12.0);
    // (-10 - 1) * 2 = -22
    EXPECT_DOUBLE_EQ(calibrated.accel_y, -22.0);
    // (-15 - 1) * 2 = -32
    EXPECT_DOUBLE_EQ(calibrated.accel_z, -32.0);

    // (-0.5 - 0.1) * 2 = -1.2
    EXPECT_DOUBLE_EQ(calibrated.gyro_x, -1.2);
    // (-1.0 - 0.1) * 2 = -2.2
    EXPECT_DOUBLE_EQ(calibrated.gyro_y, -2.2);
    // (-1.5 - 0.1) * 2 = -3.2
    EXPECT_DOUBLE_EQ(calibrated.gyro_z, -3.2);
}

// ============================================================================
// Real-World Scenario Tests
// ============================================================================

TEST(ImuCalibrationTest, GravityCompensation)
{
    // Simulate sensor reading with gravity on Z-axis and small bias
    ImuData raw;
    raw.accel_x = 0.15;   // small bias
    raw.accel_y = -0.08;  // small bias
    raw.accel_z = 9.75;   // gravity with scale error

    // Calibration to remove bias and correct scale
    ImuData calibrated = apply_calibration(
        raw,
        1.0, 1.0, 1.006,   // Z scale correction
        0.15, -0.08, 0.0,  // remove X,Y bias
        1.0, 1.0, 1.0,
        0.0, 0.0, 0.0);

    // After calibration: X,Y should be ~0, Z should be ~9.81
    EXPECT_NEAR(calibrated.accel_x, 0.0, 1e-10);
    EXPECT_NEAR(calibrated.accel_y, 0.0, 1e-10);
    EXPECT_NEAR(calibrated.accel_z, 9.8085, 0.001);  // 9.75 * 1.006
}

TEST(ImuCalibrationTest, GyroDriftCompensation)
{
    // Simulate stationary sensor with gyro drift
    ImuData raw;
    raw.gyro_x = 0.005;   // small drift
    raw.gyro_y = -0.003;  // small drift
    raw.gyro_z = 0.002;   // small drift

    // Calibration to remove drift
    ImuData calibrated = apply_calibration(
        raw,
        1.0, 1.0, 1.0,
        0.0, 0.0, 0.0,
        1.0, 1.0, 1.0,
        0.005, -0.003, 0.002);  // compensate drift

    // After calibration: gyro should read ~0 when stationary
    EXPECT_NEAR(calibrated.gyro_x, 0.0, 1e-10);
    EXPECT_NEAR(calibrated.gyro_y, 0.0, 1e-10);
    EXPECT_NEAR(calibrated.gyro_z, 0.0, 1e-10);
}


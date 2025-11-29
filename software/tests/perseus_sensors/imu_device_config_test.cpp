/**
 * @file imu_device_config_test.cpp
 * @brief Unit tests for IMU device configuration registry
 *
 * Tests the ImuDeviceRegistry class and ImuDeviceConfig structures
 * to ensure correct device configurations for all supported IMU sensors.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <algorithm>
#include "i2c_imu_driver/imu_device_config.hpp"

using namespace i2c_imu_driver;

// ============================================================================
// ImuDeviceRegistry Tests
// ============================================================================

class ImuDeviceRegistryTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        supported_devices = ImuDeviceRegistry::get_supported_devices();
    }

    std::vector<std::string> supported_devices;
};

TEST_F(ImuDeviceRegistryTest, GetSupportedDevicesReturnsExpectedCount)
{
    // Should have exactly 4 supported devices
    EXPECT_EQ(supported_devices.size(), 4)
        << "Expected 4 supported IMU devices";
}

TEST_F(ImuDeviceRegistryTest, GetSupportedDevicesContainsExpectedDevices)
{
    std::vector<std::string> expected = {"lsm6dsox", "mpu6050", "mpu9250", "lsm6ds3"};

    for (const auto& device : expected)
    {
        auto it = std::find(supported_devices.begin(), supported_devices.end(), device);
        EXPECT_NE(it, supported_devices.end())
            << "Expected device '" << device << "' not found in supported devices";
    }
}

TEST_F(ImuDeviceRegistryTest, GetDefaultDeviceReturnsLsm6dsox)
{
    EXPECT_EQ(ImuDeviceRegistry::get_default_device(), "lsm6dsox");
}

TEST_F(ImuDeviceRegistryTest, GetDeviceConfigReturnsValidConfigForAllDevices)
{
    for (const auto& device_name : supported_devices)
    {
        auto config_opt = ImuDeviceRegistry::get_device_config(device_name);
        ASSERT_TRUE(config_opt.has_value())
            << "get_device_config returned nullopt for '" << device_name << "'";

        const auto& config = config_opt->get();
        EXPECT_EQ(config.name, device_name)
            << "Config name mismatch for device '" << device_name << "'";
    }
}

TEST_F(ImuDeviceRegistryTest, GetDeviceConfigReturnsNulloptForUnknownDevice)
{
    auto config_opt = ImuDeviceRegistry::get_device_config("unknown_device");
    EXPECT_FALSE(config_opt.has_value())
        << "Expected nullopt for unknown device";

    auto config_opt2 = ImuDeviceRegistry::get_device_config("");
    EXPECT_FALSE(config_opt2.has_value())
        << "Expected nullopt for empty device name";
}

// ============================================================================
// Device Configuration Validation Tests
// ============================================================================

class ImuDeviceConfigValidationTest : public ::testing::TestWithParam<std::string>
{
protected:
    void SetUp() override
    {
        auto config_opt = ImuDeviceRegistry::get_device_config(GetParam());
        ASSERT_TRUE(config_opt.has_value());
        config = &config_opt->get();
    }

    const ImuDeviceConfig* config;
};

TEST_P(ImuDeviceConfigValidationTest, ScaleFactorsAreNonZero)
{
    EXPECT_NE(config->accel_scale_factor, 0.0)
        << "Accel scale factor should not be zero for " << GetParam();
    EXPECT_NE(config->gyro_scale_factor, 0.0)
        << "Gyro scale factor should not be zero for " << GetParam();
    EXPECT_NE(config->temp_scale_factor, 0.0)
        << "Temp scale factor should not be zero for " << GetParam();
}

TEST_P(ImuDeviceConfigValidationTest, ScaleFactorsArePositive)
{
    EXPECT_GT(config->accel_scale_factor, 0.0)
        << "Accel scale factor should be positive for " << GetParam();
    EXPECT_GT(config->gyro_scale_factor, 0.0)
        << "Gyro scale factor should be positive for " << GetParam();
    EXPECT_GT(config->temp_scale_factor, 0.0)
        << "Temp scale factor should be positive for " << GetParam();
}

TEST_P(ImuDeviceConfigValidationTest, I2cAddressIsValid)
{
    // I2C 7-bit addresses are 0x08 to 0x77
    EXPECT_GE(config->default_address, 0x08)
        << "Default address too low for " << GetParam();
    EXPECT_LE(config->default_address, 0x77)
        << "Default address too high for " << GetParam();
}

TEST_P(ImuDeviceConfigValidationTest, DataBytesPerAxisIsValid)
{
    // IMU data is typically 2 bytes (16-bit) per axis
    EXPECT_EQ(config->data_bytes_per_axis, 2)
        << "Expected 2 bytes per axis for " << GetParam();
}

TEST_P(ImuDeviceConfigValidationTest, WhoAmIRegisterAndValueAreSet)
{
    // WHO_AM_I register is used to identify the device
    EXPECT_NE(config->who_am_i_value, 0x00)
        << "WHO_AM_I value should not be 0x00 for " << GetParam();
}

INSTANTIATE_TEST_SUITE_P(
    AllSupportedDevices,
    ImuDeviceConfigValidationTest,
    ::testing::Values("lsm6dsox", "mpu6050", "mpu9250", "lsm6ds3"),
    [](const testing::TestParamInfo<std::string>& info)
    {
        return info.param;
    });

// ============================================================================
// Device Family Tests (LSM vs MPU)
// ============================================================================

class LsmDeviceTest : public ::testing::TestWithParam<std::string>
{
};

TEST_P(LsmDeviceTest, LsmDevicesAreLittleEndian)
{
    auto config_opt = ImuDeviceRegistry::get_device_config(GetParam());
    ASSERT_TRUE(config_opt.has_value());
    EXPECT_TRUE(config_opt->get().little_endian)
        << GetParam() << " should be little-endian";
}

TEST_P(LsmDeviceTest, LsmDevicesHaveCorrectAddress)
{
    auto config_opt = ImuDeviceRegistry::get_device_config(GetParam());
    ASSERT_TRUE(config_opt.has_value());
    EXPECT_EQ(config_opt->get().default_address, 0x6A)
        << GetParam() << " should have default address 0x6A";
}

INSTANTIATE_TEST_SUITE_P(
    LsmDevices,
    LsmDeviceTest,
    ::testing::Values("lsm6dsox", "lsm6ds3"));

class MpuDeviceTest : public ::testing::TestWithParam<std::string>
{
};

TEST_P(MpuDeviceTest, MpuDevicesAreBigEndian)
{
    auto config_opt = ImuDeviceRegistry::get_device_config(GetParam());
    ASSERT_TRUE(config_opt.has_value());
    EXPECT_FALSE(config_opt->get().little_endian)
        << GetParam() << " should be big-endian";
}

TEST_P(MpuDeviceTest, MpuDevicesHaveCorrectAddress)
{
    auto config_opt = ImuDeviceRegistry::get_device_config(GetParam());
    ASSERT_TRUE(config_opt.has_value());
    EXPECT_EQ(config_opt->get().default_address, 0x68)
        << GetParam() << " should have default address 0x68";
}

INSTANTIATE_TEST_SUITE_P(
    MpuDevices,
    MpuDeviceTest,
    ::testing::Values("mpu6050", "mpu9250"));

// ============================================================================
// Scale Factor Accuracy Tests
// ============================================================================

TEST(ImuScaleFactorTest, Lsm6dsoxAccelScaleIsCorrect)
{
    auto config_opt = ImuDeviceRegistry::get_device_config("lsm6dsox");
    ASSERT_TRUE(config_opt.has_value());

    // LSM6DSOX at ±2g: 0.061 mg/LSB * 9.81 m/s² / 1000
    double expected = 0.061e-3 * 9.81;
    EXPECT_NEAR(config_opt->get().accel_scale_factor, expected, 1e-10)
        << "LSM6DSOX accel scale factor should be " << expected;
}

TEST(ImuScaleFactorTest, Mpu6050AccelScaleIsCorrect)
{
    auto config_opt = ImuDeviceRegistry::get_device_config("mpu6050");
    ASSERT_TRUE(config_opt.has_value());

    // MPU6050 at ±2g: 16384 LSB/g, so scale = 9.81 / 16384
    double expected = 9.81 / 16384.0;
    EXPECT_NEAR(config_opt->get().accel_scale_factor, expected, 1e-10)
        << "MPU6050 accel scale factor should be " << expected;
}

TEST(ImuScaleFactorTest, GyroScaleConvertsToRadPerSec)
{
    // All gyro scale factors should convert raw values to rad/s
    auto devices = ImuDeviceRegistry::get_supported_devices();

    for (const auto& device : devices)
    {
        auto config_opt = ImuDeviceRegistry::get_device_config(device);
        ASSERT_TRUE(config_opt.has_value());

        // At ±250 dps full scale, max raw value gives ~4.36 rad/s
        // So scale factor should be less than 0.001 (very small)
        EXPECT_LT(config_opt->get().gyro_scale_factor, 0.001)
            << device << " gyro scale factor seems too large for rad/s output";
        EXPECT_GT(config_opt->get().gyro_scale_factor, 1e-6)
            << device << " gyro scale factor seems too small";
    }
}

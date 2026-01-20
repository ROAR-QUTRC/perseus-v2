#pragma once
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <string>
class ProcessingPlant : public rclcpp::Node
{
public:
    explicit ProcessingPlant(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ProcessingPlant();

private:
    enum class _i2c_address : uint8_t
    {
        MAGNETOMETER = 0x01,  // TODO: CHANGE THIS TO PROPER ADDRESS
        SPECTRAL = 0x39,
    };

    enum _spectral_register_address : uint8_t
    {
        ENABLE = 0x80,
        STATUS_2 = 0x90,
        STATUS_3 = 0x91,
        STATUS_1 = 0x93,
        A_STATUS = 0x94,

        // DATA
        DATA_0_L = 0x95,
        DATA_0_H = 0x96,
        DATA_1_L = 0x97,
        DATA_1_H = 0x98,
        DATA_2_L = 0x99,
        DATA_2_H = 0x9A,
        DATA_3_L = 0x9B,
        DATA_3_H = 0x9C,
        DATA_4_L = 0x9D,
        DATA_4_H = 0x9E,
        DATA_5_L = 0x9F,
        DATA_5_H = 0xA0,
        DATA_6_L = 0xA1,
        DATA_6_H = 0xA2,
        DATA_7_L = 0xA3,
        DATA_7_H = 0xA4,
        DATA_8_L = 0xA5,
        DATA_8_H = 0xA6,
        DATA_9_L = 0xA7,
        DATA_9_H = 0xA8,
        DATA_10_L = 0xA9,
        DATA_10_H = 0xAA,
        DATA_11_L = 0xAB,
        DATA_11_H = 0xAC,
        DATA_12_L = 0xAD,
        DATA_12_H = 0xAE,
        DATA_13_L = 0xAF,
        DATA_13_H = 0xB0,
        DATA_14_L = 0xB1,
        DATA_14_H = 0xB2,
        DATA_15_L = 0xB3,
        DATA_15_H = 0xB4,
        DATA_16_L = 0xB5,
        DATA_16_H = 0xB6,
        DATA_17_L = 0xB7,
        DATA_17_H = 0xB8,

        STATUS_5 = 0xBB,
        STATUS_4 = 0xBC,
        CFG_0 = 0xBF,
        CFG_1 = 0xC6,
        LED = 0xCD,
        PERS = 0xCF,
        CFG_20 = 0xDC,
        AZ_STATUS = 0xDE,
        FD_STATUS = 0xE3,
        CTRL = 0xFA,
        INTENAB = 0xF9,
    };
    enum _spectral_enable : uint8_t
    {
        FLICKER_DETECTION_ENABLE = (uint8_t)0b1 << 6,
        SMUX_ENABLE = (uint8_t)0b1 << 4,
        WAIT_ENABLE = (uint8_t)0b1 << 3,
        MEASUREMENT_ENABLE = (uint8_t)0b1 << 1,
        POWER_ON = (uint8_t)0b1 << 0,
    };

    enum _spectral_ctrl : uint8_t
    {
        SOFTWARE_RESET = (uint8_t)0b1 << 3,
        MANUAL_AUTO_ZERO = (uint8_t)0b1 << 2,
        FIFO_CLEAR = (uint8_t)0b1 << 1,
        CLEAR_SELF = (uint8_t)0b1 << 0,
    };

    enum _spectral_gain : uint8_t
    {
        GAIN_0_5 = 0,
        GAIN_1 = 1,
        GAIN_2 = 2,
        GAIN_4 = 3,
        GAIN_8 = 4,
        GAIN_16 = 5,
        GAIN_32 = 6,
        GAIN_64 = 7,
        GAIN_128 = 8,
        GAIN_256 = 9,
        GAIN_512 = 10,
        GAIN_1024 = 11,
        GAIN_2048 = 12,
    };

    enum _spectral_status_4 : uint8_t
    {
        FIFO_OVERFLOW = static_cast<uint8_t>(0b1 << 7),
        OVER_TEMP = static_cast<uint8_t>(0b1 << 5),
        FLICKER_DETECT_TRIGGER_ERROR = static_cast<uint8_t>(0b1 << 4),
        SPECTRAL_TRIGGER_ERROR = static_cast<uint8_t>(0b1 << 2),
        SLEEP_AFTER_INTERRUPT_ACTIVE = static_cast<uint8_t>(0b1 << 1),
        INITIALIZATION_BUSY = static_cast<uint8_t>(0b1 << 0),
    };

    enum _interrupt_enable : uint8_t
    {
        SATURATION_INTERRUPT_ENABLE = static_cast<uint8_t>(0b1 << 7),
        SPECTRAL_INTERRUPT_ENABLE = static_cast<uint8_t>(0b1 << 3),
        FIFO_FULL_INTERRUPT_ENABLE = static_cast<uint8_t>(0b1 << 2),
        SYSTEM_INTERRUPT_ENABLE = static_cast<uint8_t>(0b1 << 0),
    };

    void _init_i2c();
    int _check_sensor(const _i2c_address address);
    void _init_spectral();
    void _init_magnetometer();
    void _write_single_i2c(const _i2c_address address, std::vector<uint8_t> data);
    void _write_multiple_i2c(std::vector<std::pair<const _i2c_address, std::vector<uint8_t>>> messages);
    std::vector<uint8_t> _read_write_i2c(const _i2c_address address, std::vector<uint8_t> send_data, uint16_t read_bytes);
    void _read_spectral();
    void _read_magnetometer();
    void _sensor_callback();
    constexpr static auto SENSOR_READ_TIMEOUT = std::chrono::milliseconds(300);
    rclcpp::TimerBase::SharedPtr _sensor_timer;

    const uint8_t _magnetometer_enable_data = 0b11110000;
    const uint8_t _magnetometer_enable_xyz = 0b00000111;
    const uint8_t _magnetometer_start = 0x3E;
    const uint8_t _magnetometer_read = 0x4E;
    std::string _i2c_filename;
    int _i2c_file = -1;
};
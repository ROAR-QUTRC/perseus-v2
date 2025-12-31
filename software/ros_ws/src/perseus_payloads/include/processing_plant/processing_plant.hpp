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
    void _init_i2c();
    int _check_sensor(const uint16_t address);
    void _init_spectral();
    void _init_magnetometer();
    void _write_i2c(const uint16_t address, std::vector<uint8_t>* data);
    std::vector<uint8_t> _read_write_i2c(const uint16_t address, std::vector<uint8_t>* send_data, uint16_t read_bytes);
    const uint8_t _spectral_address = 0x00;  // TODO: Change this to the proper address
    const uint8_t _spectral_enable_register = 0x80;
    const uint8_t _spectral_enable_data = 0b00001011;
    const uint8_t _spectral_data_N_L_register = 0x95;
    const uint8_t _magnetometer_address = 0x01;  // TODO: Change this to the proper address
    const uint8_t _magnetometer_enable_data = 0b11110000;
    const uint8_t _magnetometer_enable_xyz = 0b00000111;
    const std::string _i2c_filename = "/dev/i2c-0";  // TODO: Make this a ROS2 input option
    int _i2c_file = -1;
};
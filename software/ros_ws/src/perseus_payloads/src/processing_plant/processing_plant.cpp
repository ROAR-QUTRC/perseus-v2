// Run this node on a raspberry pi or a device that has an i2c bus

#include "processing_plant/processing_plant.hpp"

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>

ProcessingPlant::ProcessingPlant(const rclcpp::NodeOptions& options)
    : Node("processing_plant", options)
{
    _init_i2c();
    if (_check_sensor(_spectral_address) | _check_sensor(_magnetometer_address))
    {
        return;
    }
    _init_spectral();
    _init_magnetometer();
    _sensor_timer = this->create_timer(SENSOR_READ_TIMEOUT, std::bind(&ProcessingPlant::_sensor_callback, this));
    RCLCPP_INFO(this->get_logger(), "Processing plant node initialised");
}

void ProcessingPlant::_init_i2c()
{
    _i2c_file = open(_i2c_filename.c_str(), O_RDWR, O_APPEND);
    if (_i2c_file < 0)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to open i2c path");
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "Opened i2c bus at device: %s", _i2c_filename.c_str());
    }
}

int ProcessingPlant::_check_sensor(const uint16_t address)
{
    unsigned long functions = 0;
    if (ioctl(_i2c_file, I2C_FUNCS, &functions) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error while sending 'get function' message to i2c address %d", address);
        return errno;
    }
    if (functions & I2C_FUNC_I2C)
    {
        RCLCPP_INFO(this->get_logger(), "Device at address %d can be used with I2C_RDWR, continuing", address);
        return 0;
    }
    else
    {  // I didn't want to have to implement the lower-level ways of doing stuff if I didn't have to. I will remove this function if both sensors behave
        RCLCPP_ERROR(this->get_logger(), "Device at address %d can NOT be used with I2C_RDWR", address);
        return 1;
    }
}

void ProcessingPlant::_init_spectral()
{
    std::vector<uint8_t> data = {_spectral_enable_register, _spectral_enable_data};
    _write_i2c(_spectral_address, &data);
    RCLCPP_DEBUG(this->get_logger(), "Spectral sensor initialised");
}
void ProcessingPlant::_init_magnetometer()
{
    std::vector<uint8_t> data = {_magnetometer_enable_data};
    _write_i2c(_magnetometer_address, &data);
    RCLCPP_DEBUG(this->get_logger(), "Magnetometer initialised");
}

void ProcessingPlant::_write_i2c(const uint16_t address, std::vector<uint8_t>* data)
{
    i2c_msg message[] = {{
        .addr = address,
        .flags = 0,
        .len = static_cast<uint16_t>(data->size()),
        .buf = data->data(),
    }};
    i2c_rdwr_ioctl_data send_message = {
        .msgs = message,
        .nmsgs = 1,
    };
    ioctl(_i2c_file, I2C_RDWR, &send_message);
}

std::vector<uint8_t> ProcessingPlant::_read_write_i2c(const uint16_t address, std::vector<uint8_t>* send_data, uint16_t read_bytes)
{
    uint8_t* received_data = (uint8_t*)malloc(read_bytes * sizeof(uint8_t));
    i2c_msg messages[] = {
        {.addr = address,
         .flags = 0,
         .len = static_cast<uint16_t>(send_data->size()),
         .buf = send_data->data()},
        {.addr = address,
         .flags = I2C_M_RD,
         .len = read_bytes,
         .buf = received_data}};
    i2c_rdwr_ioctl_data send_messages = {
        .msgs = messages,
        .nmsgs = sizeof(messages) / sizeof(messages[0]),
    };
    if (ioctl(_i2c_file, I2C_RDWR, &send_messages) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading from and writing to i2c bus");
        return std::vector<uint8_t>{};
    }
    std::vector<uint8_t> return_data;
    return_data.assign(received_data, (received_data + (read_bytes * sizeof(uint8_t))));
    free(received_data);
    return return_data;
}

void ProcessingPlant::_read_spectral()
{
    std::vector<uint8_t> send_data = {};
    std::vector<uint8_t> receive_data = _read_write_i2c(_spectral_address, &send_data, 36);
}

void ProcessingPlant::_read_magnetometer()
{
    std::vector<uint8_t> send_data = {_magnetometer_start};
    std::vector<uint8_t> receive_data = _read_write_i2c(_magnetometer_address, &send_data, 9);
    uint8_t status = receive_data.at(0);
    int16_t x = (receive_data.at(1) << 8) | receive_data.at(2);
    int16_t y = (receive_data.at(3) << 8) | receive_data.at(4);
    int16_t z = (receive_data.at(5) << 8) | receive_data.at(6);
    int16_t t = (receive_data.at(7) << 8) | receive_data.at(8);
    RCLCPP_INFO(this->get_logger(), "Magnetometer");
    RCLCPP_INFO(this->get_logger(), "Status: %d", status);
    RCLCPP_INFO(this->get_logger(), "X: %d", x);
    RCLCPP_INFO(this->get_logger(), "Y: %d", y);
    RCLCPP_INFO(this->get_logger(), "Z: %d", z);
    RCLCPP_INFO(this->get_logger(), "T: %d", t);
}

void ProcessingPlant::_sensor_callback()
{
    _read_magnetometer();
    _read_spectral();
}

ProcessingPlant::~ProcessingPlant()
{
    if (_i2c_file >= 0)
    {
        if (close(_i2c_file) < 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to close i2c bus at device: %s", _i2c_filename.c_str());
        }
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<ProcessingPlant>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting bucket driver node");
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running bucket driver: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
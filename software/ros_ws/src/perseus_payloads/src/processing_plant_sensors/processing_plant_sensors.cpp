// Run this node on a raspberry pi or a device that has an i2c bus

#include "processing_plant_sensors/processing_plant_sensors.hpp"

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>

ProcessingPlant::ProcessingPlant(const rclcpp::NodeOptions& options)
    : Node("processing_plant", options)
{
    _i2c_filename = this->declare_parameter("i2c_bus", "/dev/i2c-0");
    _init_i2c();
    if (_check_sensor(_i2c_address::SPECTRAL) | _check_sensor(_i2c_address::MAGNETOMETER))
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

int ProcessingPlant::_check_sensor(const _i2c_address address)
{
    unsigned long functions = 0;
    if (ioctl(_i2c_file, I2C_FUNCS, &functions) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error while sending 'get function' message to i2c address %d", static_cast<int>(address));
        return errno;
    }
    if (functions & I2C_FUNC_I2C)
    {
        RCLCPP_INFO(this->get_logger(), "Device at address %d can be used with I2C_RDWR, continuing", static_cast<int>(address));
        return 0;
    }
    else
    {  // I didn't want to have to implement the lower-level ways of doing stuff if I didn't have to. I will remove this function if both sensors behave. The spectral sensor follows the NCP standard, so it should be fine.
        RCLCPP_ERROR(this->get_logger(), "Device at address %d can NOT be used with I2C_RDWR", static_cast<int>(address));
        return 1;
    }
}

void ProcessingPlant::_init_spectral()
{
    _write_single_i2c(_i2c_address::SPECTRAL, std::vector<uint8_t>{_spectral_register_address::ENABLE, _spectral_enable::POWER_ON});
    while (_read_write_i2c(_i2c_address::SPECTRAL, std::vector<uint8_t>{_spectral_register_address::STATUS_4}, 1).front() & _spectral_status_4::INITIALIZATION_BUSY);
    std::vector<std::pair<const _i2c_address, std::vector<uint8_t>>> data = {
        {_i2c_address::SPECTRAL, {_spectral_register_address::INTENAB, (_interrupt_enable::FIFO_FULL_INTERRUPT_ENABLE | _interrupt_enable::SPECTRAL_INTERRUPT_ENABLE | _interrupt_enable::SYSTEM_INTERRUPT_ENABLE)}},
        {_i2c_address::SPECTRAL, {_spectral_register_address::PERS, 13}},
        {_i2c_address::SPECTRAL, {_spectral_register_address::CFG_1, _spectral_gain::GAIN_256}},
        {_i2c_address::SPECTRAL, {}}};
    _write_multiple_i2c(data);
    RCLCPP_DEBUG(this->get_logger(), "Spectral sensor initialised");
}
void ProcessingPlant::_init_magnetometer()
{
    _write_single_i2c(_i2c_address::MAGNETOMETER, std::vector<uint8_t>{_magnetometer_enable_data});
    RCLCPP_DEBUG(this->get_logger(), "Magnetometer initialised");
}

void ProcessingPlant::_write_single_i2c(const _i2c_address address, std::vector<uint8_t> data)
{
    i2c_msg message[] = {{
        .addr = static_cast<uint16_t>(static_cast<uint8_t>(address) << 1),
        .flags = 0,
        .len = static_cast<uint16_t>(data.size()),
        .buf = data.data(),
    }};
    i2c_rdwr_ioctl_data send_message = {
        .msgs = message,
        .nmsgs = 1,
    };
    ioctl(_i2c_file, I2C_RDWR, &send_message);
}

void ProcessingPlant::_write_multiple_i2c(std::vector<std::pair<const _i2c_address, std::vector<uint8_t>>> messages)
{
    std::vector<i2c_msg> messages_formatted = {};
    for (std::pair<const _i2c_address, std::vector<uint8_t>> message : messages)
    {
        messages_formatted.emplace_back(i2c_msg{
            .addr = static_cast<uint16_t>(static_cast<uint8_t>(message.first) << 1),
            .flags = 0,
            .len = static_cast<uint16_t>(message.second.size()),
            .buf = message.second.data(),
        });
    }
    i2c_rdwr_ioctl_data send_message = {
        .msgs = messages_formatted.data(),
        .nmsgs = static_cast<uint32_t>(messages_formatted.size()),
    };
    ioctl(_i2c_file, I2C_RDWR, &send_message);
}

std::vector<uint8_t> ProcessingPlant::_read_write_i2c(const _i2c_address address, std::vector<uint8_t> send_data, uint16_t read_bytes)
{
    uint8_t* received_data = (uint8_t*)malloc(read_bytes * sizeof(uint8_t));
    i2c_msg messages[] = {
        {.addr = static_cast<uint16_t>(static_cast<uint8_t>(address) << 1),
         .flags = 0,
         .len = static_cast<uint16_t>(send_data.size()),
         .buf = send_data.data()},
        {.addr = static_cast<uint16_t>(static_cast<uint8_t>(address) << 1 | 0b1),
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
    std::vector<uint8_t> receive_data = _read_write_i2c(_i2c_address::SPECTRAL, send_data, 36);
}

void ProcessingPlant::_read_magnetometer()
{
    std::vector<uint8_t> receive_data = _read_write_i2c(_i2c_address::MAGNETOMETER, std::vector<uint8_t>{_magnetometer_start}, 9);
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
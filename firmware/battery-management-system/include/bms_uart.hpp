#include <chrono>
#include <numeric>
#include <vector>

#include "driver/uart.h"

class BmsUartMessage
{
public:
    enum class status_byte_t : uint8_t
    {
        READ = 0xA5,
        WRITE = 0x5A,
    };
    enum class command_byte_t : uint8_t
    {
        INFORMATION = 0x03,
        VOLTAGE = 0x04,
        VERSION = 0x05,
        MOSFET_CONTROL = 0xE1,
    };
    BmsUartMessage(command_byte_t command, std::vector<uint8_t>& data);
    uint8_t get_start_byte() { return _start_byte; }
    status_byte_t get_status_byte() { return _status_byte; }
    command_byte_t get_command_byte() { return _command_byte; }
    uint8_t get_data_length() { return _data_length; }
    std::vector<uint8_t> get_data() { return _data; };
    uint16_t get_checksum() { return _checksum; }
    uint8_t get_stop_byte() { return _stop_byte; }

private:
    uint16_t _convert_to_twos_complement(uint16_t number);
    uint8_t _start_byte = 0xDD;
    status_byte_t _status_byte;
    command_byte_t _command_byte;
    uint8_t _data_length = 0;
    std::vector<uint8_t> _data;
    uint16_t _checksum;
    uint8_t _stop_byte = 0x77;
};

class BmsUartDriver
{
public:
    BmsUartDriver(uart_port_t port_number);
    ~BmsUartDriver();
    const std::chrono::steady_clock::duration get_voltage_interval()
    {
        return _voltage_interval;
    }
    const std::chrono::steady_clock::duration get_information_interval()
    {
        return _information_interval;
    }
    bool transmit(BmsUartMessage& message);
    bool receive(BmsUartMessage& message);

private:
    const int BUFFER_SIZE = 1024;
    uart_port_t _uart_port_number;
    uart_config_t _uart_config = {
        // BMS is 9600 baud 8N1
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
    };
    const std::chrono::steady_clock::duration _voltage_interval = std::chrono::milliseconds(100);
    const std::chrono::steady_clock::duration _information_interval = std::chrono::milliseconds(100);
};

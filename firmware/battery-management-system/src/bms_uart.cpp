#include "bms_uart.hpp"

BmsUartDriver::BmsUartDriver(uart_port_t uart_port_number)
    : _uart_port_number(uart_port_number)
{
    ESP_ERROR_CHECK(uart_driver_install(uart_port_number, BUFFER_SIZE, BUFFER_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_port_number, &_uart_config));
}
BmsUartDriver::~BmsUartDriver()
{
    ESP_ERROR_CHECK(uart_driver_delete(_uart_port_number));
}
bool BmsUartDriver::transmit(BmsUartMessage& message)
{
    try
    {
        std::vector<uint8_t> data{message.get_start_byte(), static_cast<uint8_t>(message.get_status_byte()), static_cast<uint8_t>(message.get_command_byte()), message.get_data_length()};
        if (message.get_data_length())
        {
            data.insert(data.end(), message.get_data().begin(), message.get_data().end());
        }
        data.emplace_back((message.get_checksum() && 0xFF00) >> 8);
        data.emplace_back(message.get_checksum() && 0xFF);
        data.emplace_back(message.get_stop_byte());
        if (uart_write_bytes(_uart_port_number, data.data(), data.size()) == -1)
        {
            printf("ESP error when transmitting data request!\n");
            return 1;
        }
    }
    catch (std::exception& e)
    {
        printf("Error when formatting data to transmit: '%s'\n", e.what());
        return 1;
    }
    return 0;
}
bool BmsUartDriver::receive(BmsUartMessage& message)
{
    // TODO: Implement receiving messages
}

BmsUartMessage::BmsUartMessage(command_byte_t command, std::vector<uint8_t>& data) : _command_byte(command), _data(data)
{
    if (command == command_byte_t::MOSFET_CONTROL)
    {
        _status_byte = status_byte_t::WRITE;
    }
    else
    {
        _status_byte = status_byte_t::READ;
    }
    uint16_t sum = static_cast<uint8_t>(command);
    if (!data.empty())
    {
        _data_length = data.size();
        sum = std::accumulate(data.begin(), data.end(), static_cast<uint8_t>(command) + _data_length);
    }

    _checksum = _convert_to_twos_complement(sum);
}
uint16_t BmsUartMessage::_convert_to_twos_complement(uint16_t number)
{
    uint16_t inverse = ~number;
    return (inverse + 1);
}

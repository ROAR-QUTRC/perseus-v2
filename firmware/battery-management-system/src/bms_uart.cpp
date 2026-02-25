#include "bms_uart.hpp"

BmsUartDriver::BmsUartDriver(uart_port_t uart_port_number) : _uart_port_number(uart_port_number)
{
    ESP_ERROR_CHECK(uart_driver_install(uart_port_number, BUFFER_SIZE, BUFFER_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_port_number, &_uart_config));
}
BmsUartDriver::~BmsUartDriver()
{
    ESP_ERROR_CHECK(uart_driver_delete(_uart_port_number));
}
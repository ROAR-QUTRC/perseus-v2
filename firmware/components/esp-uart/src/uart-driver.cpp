#include "uart-driver.hpp"

#include <driver/uart.h>

#include <iterator>
#include <stdexcept>

#include "freertos/FreeRTOS.h"

TickType_t ticks_to_wait_receive = 100;

UartDriver::UartDriver(const unsigned int port_number, const unsigned int baud_rate, const unsigned int buffer_size)
{
    if (SOC_UART_NUM <= port_number)
    {
        throw std::invalid_argument("Error: Port number is too high!");
        return;
    }
    _uart_port = port_number;
    ESP_ERROR_CHECK(uart_driver_install(static_cast<uart_port_t>(port_number), buffer_size, buffer_size, 0, NULL, 0));
    uart_config_t uart_config{.baud_rate = static_cast<int>(baud_rate), .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .rx_flow_ctrl_thresh = 0};
    ESP_ERROR_CHECK(uart_param_config(static_cast<uart_port_t>(port_number), &uart_config));
}

UartDriver::~UartDriver()
{
    ESP_ERROR_CHECK(uart_driver_delete(static_cast<uart_port_t>(_uart_port)));
}

void UartDriver::transmit(raw_uart_message_t& message)
{
    uart_write_bytes(static_cast<uart_port_t>(_uart_port), static_cast<std::vector<uint8_t>>(message).data(), static_cast<std::vector<uint8_t>>(message).size());
}

void UartDriver::receive(raw_uart_message_t& message, unsigned int bytes_to_read)
{
    size_t bytes_available = 0;
    while (bytes_available < bytes_to_read)
    {
        ESP_ERROR_CHECK(uart_get_buffered_data_len(static_cast<uart_port_t>(_uart_port), &bytes_available));
    }
    uint8_t received_data[bytes_to_read] = {};
    uart_read_bytes(static_cast<uart_port_t>(_uart_port), received_data, bytes_to_read, ticks_to_wait_receive);
    std::copy(received_data, received_data + bytes_to_read, std::back_inserter(message));
}

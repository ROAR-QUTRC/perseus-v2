#include "uart-driver.hpp"

#include <driver/uart.h>

#include <chrono>
#include <cstdint>
#include <iterator>
#include <stdexcept>
#include <vector>

#include "freertos/FreeRTOS.h"
using namespace std::chrono_literals;

TickType_t ticks_to_wait_receive = 100;

UartDriver::UartDriver(const unsigned int port_number, const unsigned int baud_rate, const unsigned int buffer_size)
{
    if (SOC_UART_NUM <= port_number)
    {
        throw std::invalid_argument("Error: Port number is too high!");
        return;
    }
    _uart_port = port_number;
    if ((uart_driver_install(static_cast<uart_port_t>(port_number), buffer_size, buffer_size, 0, NULL, 0)) != ESP_OK)
    {
        throw std::runtime_error("Error: Can't install uart driver\n");
        return;
    }
    uart_config_t uart_config{.baud_rate = static_cast<int>(baud_rate), .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .rx_flow_ctrl_thresh = 0};
    if ((uart_param_config(static_cast<uart_port_t>(port_number), &uart_config)) != ESP_OK)
    {
        throw std::runtime_error("Error: Can't configure uart parameters");
    }
}

UartDriver::~UartDriver()
{
    ESP_ERROR_CHECK(uart_driver_delete(static_cast<uart_port_t>(_uart_port)));
}

void UartDriver::transmit(raw_uart_message_t message)
{
    uart_flush(static_cast<uart_port_t>(_uart_port));
    uart_write_bytes(static_cast<uart_port_t>(_uart_port), static_cast<std::vector<uint8_t>>(message).data(), static_cast<std::vector<uint8_t>>(message).size());
}

int UartDriver::receive(raw_uart_message_t& message, unsigned int bytes_to_read)
{
    if (bytes_to_read == 0)
    {
        return 0;
    }
    size_t bytes_available = 0;
    TickType_t start_of_receive = xTaskGetTickCount();
    while (bytes_available < bytes_to_read)
    {
        printf("No full message yet - bytes available: %d\n", bytes_available);
        ESP_ERROR_CHECK(uart_get_buffered_data_len(static_cast<uart_port_t>(_uart_port), &bytes_available));
        if ((start_of_receive - xTaskGetTickCount()) > 50)
        {
            return -1;
        }
    }
    uint8_t received_data[bytes_to_read] = {};
    uart_read_bytes(static_cast<uart_port_t>(_uart_port), received_data, bytes_to_read, ticks_to_wait_receive);
    std::copy(received_data, received_data + bytes_to_read, std::back_inserter(message));
    return 0;
}

int UartDriver::receive_flagged(raw_uart_message_t& message, unsigned int header_length, unsigned int trailer_length, flagged_uart_message_t::crc_creator_t crc_creator, unsigned int footer_length)
{
    raw_uart_message_t header = {};
    raw_uart_message_t data = {};
    raw_uart_message_t trailer = {};
    raw_uart_message_t footer = {};

    if (receive(header, header_length + 1) == -1)
    {
        return -1;
    }
    if (receive(data, header.back()) == -1)
    {
        return -1;
    }
    if (receive(trailer, trailer_length) == -1)
    {
        return -1;
    }
    std::vector<uint8_t> expected_crc = crc_creator.value()(data);
    std::vector<uint8_t> actual_crc = {};
    if (receive(actual_crc, expected_crc.size()) == -1)
    {
        return -1;
    }
    if (expected_crc != actual_crc)
    {
        return -1;
    }
    if (receive(footer, footer_length) == -1)
    {
        return -1;
    }
    message = {};
    message.insert(header.begin(), header.end(), message.end());
    message.insert(data.begin(), data.end(), message.end());
    message.insert(trailer.begin(), trailer.end(), message.end());
    message.insert(actual_crc.begin(), actual_crc.end(), message.end());
    message.insert(footer.begin(), footer.end(), message.end());
    return 0;
}

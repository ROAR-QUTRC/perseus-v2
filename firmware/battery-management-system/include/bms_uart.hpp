#include <chrono>

#include "driver/uart.h"

class BmsUartDriver
{
public:
    BmsUartDriver(uart_port_t port_number);
    ~BmsUartDriver();

private:
    int convert_to_twos_complement(int number);
    const int BUFFER_SIZE = 1024;
    uart_port_t _uart_port_number;
    uart_config_t _uart_config = {
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
#pragma once
#include "uart-message.hpp"

static const int DEFAULT_BUFFER_SIZE = 1024;

class UartDriver
{
public:
    UartDriver(const unsigned int port_number, const unsigned int baud_rate, const unsigned int buffer_size = DEFAULT_BUFFER_SIZE);
    ~UartDriver();
    void transmit(raw_uart_message_t message);
    int receive(raw_uart_message_t& message, unsigned int bytes_to_read);
    int receive_flagged(raw_uart_message_t& message, unsigned int header_length, unsigned int trailer_length, flagged_uart_message_t::crc_creator_t crc_creator, unsigned int footer_length);

private:
    unsigned int _uart_port;
};

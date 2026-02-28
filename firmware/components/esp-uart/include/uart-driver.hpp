#include "uart-message.hpp"

static const int DEFAULT_BUFFER_SIZE = 1024;

class UartDriver
{
public:
    UartDriver(const unsigned int port_number, const unsigned int baud_rate, const unsigned int buffer_size = DEFAULT_BUFFER_SIZE);
    ~UartDriver();
    void transmit(raw_uart_message_t& message);

private:
    unsigned int _uart_port;
};

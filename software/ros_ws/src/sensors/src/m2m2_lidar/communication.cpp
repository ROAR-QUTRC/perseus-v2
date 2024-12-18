#include "sensors/m2m2_lidar/communication.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

namespace sensors
{

Communication::Communication(const std::string& address, uint16_t port)
    : _address(address)
    , _port(port)
    , _socket(-1)
{
}

Communication::~Communication()
{
    if (_socket != -1)
    {
        close(_socket);
    }
}

bool Communication::initialize()
{
    _socket = socket(AF_INET, SOCK_STREAM, 0);
    if (_socket < 0)
    {
        return false;
    }

    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(_port);
    
    if (inet_pton(AF_INET, _address.c_str(), &serverAddr.sin_addr) <= 0)
    {
        close(_socket);
        _socket = -1;
        return false;
    }

    if (connect(_socket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0)
    {
        close(_socket);
        _socket = -1;
        return false;
    }

    return true;
}

bool Communication::sendCommand(const uint8_t* command, size_t length)
{
    if (_socket == -1)
    {
        return false;
    }

    ssize_t bytesSent = send(_socket, command, length, 0);
    return bytesSent == static_cast<ssize_t>(length);
}

ssize_t Communication::receiveData(uint8_t* buffer, size_t maxLength)
{
    if (_socket == -1)
    {
        return -1;
    }

    return recv(_socket, buffer, maxLength, 0);
}

} // namespace sensors

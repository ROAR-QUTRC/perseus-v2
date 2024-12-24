#include "simple_networking/client.hpp"

#include <arpa/inet.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <cstring>
#include <format>
#include <stdexcept>

using std::string;

using namespace networking;

Client::Client(const address_t& address, const socket_protocol protocol, const address_t& bindAddress, const socket_config_handlers_t& configHandlers)
    : _address(address), _bindAddress(bindAddress), _protocol(protocol)
{
    _socket = FdWrapper(std::bind(&Client::_createSocket, this, configHandlers));
}

ssize_t Client::transmit(const string& message)
{
    return transmit(std::vector<uint8_t>(message.begin(), message.end()));
}
ssize_t Client::transmit(const std::vector<uint8_t>& buffer)
{
    const ssize_t bytesWritten = send(static_cast<int>(_socket), buffer.data(), buffer.size(), 0);
    if (bytesWritten < 0)
    {
        string err = std::strerror(errno);
        throw std::runtime_error("Failed to transmit data: " + err);
    }
    if (bytesWritten == 0 && buffer.size() > 0)
    {
        throw std::runtime_error("Connection closed: 0 bytes transmitted");
    }
    return bytesWritten;
}

std::optional<std::vector<uint8_t>> Client::receive(size_t len, bool blocking)
{
    std::vector<uint8_t> buffer(len);
    const ssize_t bytesRead = recv(static_cast<int>(_socket), buffer.data(), len, blocking ? 0 : MSG_DONTWAIT);
    if (bytesRead < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return std::nullopt;
        throw std::runtime_error(std::format("Failed to receive packet from {}: {}",
                                             _getFullAddressString(), std::strerror(errno)));
    }
    if (bytesRead == 0)
    {
        if (len > 0)
            throw std::runtime_error(std::format("Connection closed: 0 bytes received from {}",
                                                 _getFullAddressString()));
        return std::nullopt;
    }

    buffer.resize(bytesRead);

    return buffer;
}

int Client::_createBoundSocket(struct addrinfo*& currentAddr, struct addrinfo* currentBindAddr, const socket_config_handlers_t& configHandlers)
{
    bool shouldSkipBind = currentBindAddr == nullptr;
    while (currentAddr && (shouldSkipBind || currentBindAddr))
    {
        int socketFd = socket(currentAddr->ai_family, currentAddr->ai_socktype, currentAddr->ai_protocol);
        if (socketFd == -1)
        {
            currentAddr = currentAddr->ai_next;
            continue;
        }
        if (currentBindAddr)
        {
            const int yes = 1;
            // enable address reuse (prevents "address already in use" errors)
            if (setsockopt(socketFd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0)
            {
                close(socketFd);
                currentBindAddr = currentBindAddr->ai_next;
                continue;
            }
            if (configHandlers.preBind)
            {
                if (!configHandlers.preBind(socketFd))
                {
                    close(socketFd);
                    currentBindAddr = currentBindAddr->ai_next;
                    continue;
                }
            }
            if (bind(socketFd, currentBindAddr->ai_addr, currentBindAddr->ai_addrlen) < 0)
            {
                close(socketFd);
                currentBindAddr = currentBindAddr->ai_next;
                continue;
            }
        }
        return socketFd;
    }
    // if we hit this, socket creation for all addresses failed
    if (!currentAddr)
    {
        throw std::runtime_error(std::format("Failed to create socket for {}: {}",
                                             _getFullAddressString(), std::strerror(errno)));
    }
    // and if we hit this, binding failed
    if (!shouldSkipBind && !currentBindAddr)
    {
        throw std::runtime_error(std::format("Failed to bind address {}: {}",
                                             static_cast<string>(_bindAddress), std::strerror(errno)));
    }
    // we should never hit this
    assert(false);
    // make the compiler happy
    return -1;
}

#include <iostream>
int Client::_createSocket(const socket_config_handlers_t& configHandlers)
{
    // perform address resolution
    struct addrinfo hints
    {
    };
    std::fill_n(reinterpret_cast<uint8_t*>(&hints), sizeof(hints), 0);
    hints.ai_family = AF_UNSPEC;  // don't care whether it's IPv4 or IPv6
    hints.ai_socktype = (_protocol == socket_protocol::TCP) ? SOCK_STREAM : SOCK_DGRAM;

    if (const int status =
            getaddrinfo(_address.hostname.c_str(), _address.service.c_str(), &hints, &_destinationAddrinfo);
        status != 0)
    {
        throw std::runtime_error(std::format("Failed to resolve address {}: {}",
                                             _getFullAddressString(),
                                             gai_strerror(status)));
    }

    if (!_bindAddress.service.empty())
    {
        hints.ai_flags = AI_PASSIVE;  // wildcard IP address - will be ignored if hostname is provided
        const char* const bindHostname = _bindAddress.hostname.empty() ? nullptr : _bindAddress.hostname.c_str();
        if (const int status =
                getaddrinfo(bindHostname, _bindAddress.service.c_str(), &hints, &_bindAddrinfo);
            status != 0)
        {
            throw std::runtime_error(std::format("Failed to resolve bind address {}: {}",
                                                 static_cast<string>(_bindAddress),
                                                 gai_strerror(status)));
        }
    }

    for (auto currentAddr = _destinationAddrinfo.get(); currentAddr; currentAddr = currentAddr->ai_next)
    {
        int socketFd = _createBoundSocket(currentAddr, _bindAddrinfo.get(), configHandlers);
        if (configHandlers.preConnect)
        {
            if (!configHandlers.preConnect(socketFd))
            {
                close(socketFd);
                // same thing here
                if (!currentAddr->ai_next)
                {
                    throw std::runtime_error(std::format("Failed to configure socket pre-connection for {}: {}",
                                                         _getFullAddressString(), std::strerror(errno)));
                }
                continue;
            }
        }
        if (connect(socketFd, _destinationAddrinfo->ai_addr, _destinationAddrinfo->ai_addrlen) < 0)
        {
            close(socketFd);
            // if we've run out of addresses to try, it's failed
            if (!currentAddr->ai_next)
            {
                throw std::runtime_error(std::format("Failed to connect to {}: {}",
                                                     _getFullAddressString(), std::strerror(errno)));
            }
            continue;
        }
        if (configHandlers.postConnect)
        {
            if (!configHandlers.postConnect(socketFd))
            {
                close(socketFd);
                // same thing here
                if (!currentAddr->ai_next)
                {
                    throw std::runtime_error(std::format("Failed to configure socket post-connection for {}: {}",
                                                         _getFullAddressString(), std::strerror(errno)));
                }
                continue;
            }
        }
        // free the lists, we don't need them any more
        _destinationAddrinfo = nullptr;
        _bindAddrinfo = nullptr;

        return socketFd;
    }
    // we should never hit this - any failure by this point should have thrown an exception
    assert(false);
    // make the compiler happy
    return -1;
}
string Client::_getFullAddressString() const
{
    return std::format("{} ({})", static_cast<string>(_address), getStringFromProtocol(_protocol));
}
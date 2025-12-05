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

Client::Client(const address_t& address, const socket_protocol protocol, const address_t& bind_address, const socket_config_handlers_t& config_handlers)
    : _address(address),
      _bind_address(bind_address),
      _protocol(protocol)
{
    _socket = FdWrapper(std::bind(&Client::_create_socket, this, config_handlers));
}

ssize_t Client::transmit(const string& message)
{
    return transmit(std::vector<uint8_t>(message.begin(), message.end()));
}
ssize_t Client::transmit(const std::vector<uint8_t>& buffer)
{
    const ssize_t bytes_written = send(static_cast<int>(_socket), buffer.data(), buffer.size(), 0);
    if (bytes_written < 0)
    {
        string err = std::strerror(errno);
        throw std::runtime_error("Failed to transmit data: " + err);
    }
    if (bytes_written == 0 && buffer.size() > 0)
    {
        throw std::runtime_error("Connection closed: 0 bytes transmitted");
    }
    return bytes_written;
}

std::optional<std::vector<uint8_t>> Client::receive(size_t len, bool blocking)
{
    std::vector<uint8_t> buffer(len);
    const ssize_t bytes_read = recv(static_cast<int>(_socket), buffer.data(), len, blocking ? 0 : MSG_DONTWAIT);
    if (bytes_read < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return std::nullopt;
        throw std::runtime_error(std::format("Failed to receive packet from {}: {}",
                                             _get_full_address_string(), std::strerror(errno)));
    }
    if (bytes_read == 0)
    {
        if (len > 0)
            throw std::runtime_error(std::format("Connection closed: 0 bytes received from {}",
                                                 _get_full_address_string()));
        return std::nullopt;
    }

    buffer.resize(bytes_read);

    return buffer;
}

int Client::_create_bound_socket(struct addrinfo*& current_addr, struct addrinfo* current_bind_addr, const socket_config_handlers_t& config_handlers)
{
    bool should_skip_bind = current_bind_addr == nullptr;
    while (current_addr && (should_skip_bind || current_bind_addr))
    {
        int socket_fd = socket(current_addr->ai_family, current_addr->ai_socktype, current_addr->ai_protocol);
        if (socket_fd == -1)
        {
            current_addr = current_addr->ai_next;
            continue;
        }
        if (current_bind_addr)
        {
            const int yes = 1;
            // enable address reuse (prevents "address already in use" errors)
            if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0)
            {
                close(socket_fd);
                current_bind_addr = current_bind_addr->ai_next;
                continue;
            }
            if (config_handlers.pre_bind)
            {
                if (!config_handlers.pre_bind(socket_fd))
                {
                    close(socket_fd);
                    current_bind_addr = current_bind_addr->ai_next;
                    continue;
                }
            }
            if (bind(socket_fd, current_bind_addr->ai_addr, current_bind_addr->ai_addrlen) < 0)
            {
                close(socket_fd);
                current_bind_addr = current_bind_addr->ai_next;
                continue;
            }
        }
        return socket_fd;
    }
    // if we hit this, socket creation for all addresses failed
    if (!current_addr)
    {
        throw std::runtime_error(std::format("Failed to create socket for {}: {}",
                                             _get_full_address_string(), std::strerror(errno)));
    }
    // and if we hit this, binding failed
    if (!should_skip_bind && !current_bind_addr)
    {
        throw std::runtime_error(std::format("Failed to bind address {}: {}",
                                             static_cast<string>(_bind_address), std::strerror(errno)));
    }
    // we should never hit this
    assert(false);
    // make the compiler happy
    return -1;
}

#include <iostream>
int Client::_create_socket(const socket_config_handlers_t& config_handlers)
{
    // perform address resolution
    struct addrinfo hints{};
    std::fill_n(reinterpret_cast<uint8_t*>(&hints), sizeof(hints), 0);
    hints.ai_family = AF_UNSPEC;  // don't care whether it's IPv4 or IPv6
    hints.ai_socktype = (_protocol == socket_protocol::TCP) ? SOCK_STREAM : SOCK_DGRAM;

    if (const int status =
            getaddrinfo(_address.hostname.c_str(), _address.service.c_str(), &hints, &_destination_addrinfo);
        status != 0)
    {
        throw std::runtime_error(std::format("Failed to resolve address {}: {}",
                                             _get_full_address_string(),
                                             gai_strerror(status)));
    }

    if (!_bind_address.service.empty())
    {
        hints.ai_flags = AI_PASSIVE;  // wildcard IP address - will be ignored if hostname is provided
        const char* const bind_hostname = _bind_address.hostname.empty() ? nullptr : _bind_address.hostname.c_str();
        if (const int status =
                getaddrinfo(bind_hostname, _bind_address.service.c_str(), &hints, &_bind_addrinfo);
            status != 0)
        {
            throw std::runtime_error(std::format("Failed to resolve bind address {}: {}",
                                                 static_cast<string>(_bind_address),
                                                 gai_strerror(status)));
        }
    }

    for (auto current_addr = _destination_addrinfo.get(); current_addr; current_addr = current_addr->ai_next)
    {
        int socket_fd = _create_bound_socket(current_addr, _bind_addrinfo.get(), config_handlers);
        if (config_handlers.pre_connect)
        {
            if (!config_handlers.pre_connect(socket_fd))
            {
                close(socket_fd);
                // same thing here
                if (!current_addr->ai_next)
                {
                    throw std::runtime_error(std::format("Failed to configure socket pre-connection for {}: {}",
                                                         _get_full_address_string(), std::strerror(errno)));
                }
                continue;
            }
        }
        if (connect(socket_fd, _destination_addrinfo->ai_addr, _destination_addrinfo->ai_addrlen) < 0)
        {
            close(socket_fd);
            // if we've run out of addresses to try, it's failed
            if (!current_addr->ai_next)
            {
                throw std::runtime_error(std::format("Failed to connect to {}: {}",
                                                     _get_full_address_string(), std::strerror(errno)));
            }
            continue;
        }
        if (config_handlers.post_connect)
        {
            if (!config_handlers.post_connect(socket_fd))
            {
                close(socket_fd);
                // same thing here
                if (!current_addr->ai_next)
                {
                    throw std::runtime_error(std::format("Failed to configure socket post-connection for {}: {}",
                                                         _get_full_address_string(), std::strerror(errno)));
                }
                continue;
            }
        }
        // free the lists, we don't need them any more
        _destination_addrinfo = nullptr;
        _bind_addrinfo = nullptr;

        return socket_fd;
    }
    // we should never hit this - any failure by this point should have thrown an exception
    assert(false);
    // make the compiler happy
    return -1;
}
string Client::_get_full_address_string() const
{
    return std::format("{} ({})", static_cast<string>(_address), get_string_from_protocol(_protocol));
}
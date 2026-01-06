#pragma once

#include <netdb.h>

#include <cstdint>
#include <fd_wrapper.hpp>
#include <functional>
#include <optional>
#include <ptr_wrapper.hpp>
#include <string>
#include <vector>

#include "core.hpp"

namespace networking
{
    /// @brief A struct to hold handlers for configuring a socket at various stages of its creation
    /// @details Each handler is called with the currently open socket file descriptor, and should return true if the operation was successful
    struct socket_config_handlers_t
    {
        /// @brief Handler to be run before binding the socket
        std::function<bool(int)> pre_bind{};
        /// @brief Handler to be run before connecting the socket
        std::function<bool(int)> pre_connect{};
        /// @brief Handler to be run after connecting the socket
        std::function<bool(int)> post_connect{};
    };
    /// @brief A RAII wrapper around network sockets providing a simple interface for sending and receiving data
    class Client
    {
    public:
        /// @brief Initialise a client socket with the given address, protocol, and service/port
        /// @param address The address to connect to
        /// @param protocol The protocol to use (TCP/UDP)
        /// @param bind_service The service/port to bind to
        /// @param config_handlers Configuration handlers for various stages of socket creation
        Client(const address_t& address,
               const socket_protocol protocol = socket_protocol::TCP,
               const address_t& bind_service = {},
               const socket_config_handlers_t& config_handlers = {});
        /// @overload Client(const address_t& address, const socket_protocol protocol, const std::string& bind_service, const socket_config_handlers_t& config_handlers)
        Client(const address_t& address,
               const socket_protocol protocol,
               const uint16_t bind_port,
               const socket_config_handlers_t& config_handlers = {})
            : Client(address, protocol, {.service = std::to_string(bind_port)}, config_handlers) {};
        // copy constructor + assignment are deleted, since copying doesn't make sense
        // however, we allow move semantics
        Client(const Client& other) = delete;
        Client(Client&& other) noexcept
        {
            swap(*this, other);
        }
        Client& operator=(Client other) = delete;
        Client& operator=(Client&& other) noexcept
        {
            swap(*this, other);
            return *this;
        }

        friend void swap(Client& first, Client& second) noexcept
        {
            using std::swap;
            swap(first._address, second._address);
            swap(first._bind_address, second._bind_address);
            swap(first._protocol, second._protocol);
            swap(first._destination_addrinfo, second._destination_addrinfo);
            swap(first._bind_addrinfo, second._bind_addrinfo);
            swap(first._socket, second._socket);
        }

        /**
         * @brief Transmit a string over the socket
         *
         * @param message Data to transmit
         * @return ssize_t Number of bytes actually transmitted
         */
        ssize_t transmit(const std::string& message);
        /**
         * @brief Transmit a buffer over the socket
         *
         * @param buffer Data to transmit
         * @return ssize_t Number of bytes actually transmitted
         */
        ssize_t transmit(const std::vector<uint8_t>& buffer);
        /**
         * @brief Receive data from the socket
         *
         * @param len Number of bytes to attempt to receive
         * @param blocking Whether or not to block until data is available
         * @return std::optional<std::vector<uint8_t>> Received data, if any
         * @note When blocking is false, this function will return std::nullopt if there isn't any data available
         */
        std::optional<std::vector<uint8_t>> receive(size_t len, bool blocking = false);

    private:
        /**
         * @brief Create a socket as per the internal parameters (that is, @ref _address, @ref _bind_address, @ref _protocol)
         *
         * @param config_handlers Configuration handlers for various stages of socket creation
         * @return int The open socket file descriptor
         */
        int _create_socket(const socket_config_handlers_t& config_handlers);
        /**
         * @brief Create a socket, optionally bound to an address and port
         *
         * @param current_addr Current address to attempt to connect to - only used for socket inet family
         * @param current_bind_addr Address to attempt to bind to (if applicable)
         * @param config_handlers Configuration handlers to run during the relevant stages
         * @return int Open socket file descriptor
         * @note @ref current_addr is passed by reference to allow updating it - it will be updated to the first address for which socket creation succeeded
         */
        int _create_bound_socket(struct addrinfo*& current_addr, struct addrinfo* current_bind_addr, const socket_config_handlers_t& config_handlers);
        /**
         * @brief Convert the address, protocol, and service/port to a string for error messages
         *
         * @return std::string String in the format "address:port (protocol)"
         */
        std::string _get_full_address_string() const;

        address_t _address{};
        address_t _bind_address{};
        socket_protocol _protocol{};

        PtrWrapper<struct addrinfo> _destination_addrinfo{nullptr, &freeaddrinfo};
        PtrWrapper<struct addrinfo> _bind_addrinfo{nullptr, &freeaddrinfo};

        FdWrapper _socket;
    };
}
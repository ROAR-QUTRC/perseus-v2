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
        std::function<bool(int)> preBind{};
        /// @brief Handler to be run before connecting the socket
        std::function<bool(int)> preConnect{};
        /// @brief Handler to be run after connecting the socket
        std::function<bool(int)> postConnect{};
    };
    /// @brief A RAII wrapper around network sockets providing a simple interface for sending and receiving data
    class Client
    {
    public:
        /// @brief Initialise a client socket with the given address, protocol, and service/port
        /// @param address The address to connect to
        /// @param protocol The protocol to use (TCP/UDP)
        /// @param bindService The service/port to bind to
        /// @param configHandlers Configuration handlers for various stages of socket creation
        Client(const address_t& address,
               const socket_protocol protocol = socket_protocol::TCP,
               const address_t& bindService = {},
               const socket_config_handlers_t& configHandlers = {});
        /// @overload Client(const address_t& address, const socket_protocol protocol, const std::string& bindService, const socket_config_handlers_t& configHandlers)
        Client(const address_t& address,
               const socket_protocol protocol,
               const uint16_t bindPort,
               const socket_config_handlers_t& configHandlers = {})
            : Client(address, protocol, {.service = std::to_string(bindPort)}, configHandlers) {};
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
            swap(first._bindAddress, second._bindAddress);
            swap(first._protocol, second._protocol);
            swap(first._destinationAddrinfo, second._destinationAddrinfo);
            swap(first._bindAddrinfo, second._bindAddrinfo);
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
         * @brief Create a socket as per the internal parameters (that is, @ref _address, @ref _bindAddress, @ref _protocol)
         *
         * @param configHandlers Configuration handlers for various stages of socket creation
         * @return int The open socket file descriptor
         */
        int _createSocket(const socket_config_handlers_t& configHandlers);
        /**
         * @brief Create a socket, optionally bound to an address and port
         *
         * @param currentAddr Current address to attempt to connect to - only used for socket inet family
         * @param currentBindAddr Address to attempt to bind to (if applicable)
         * @param configHandlers Configuration handlers to run during the relevant stages
         * @return int Open socket file descriptor
         * @note @ref currentAddr is passed by reference to allow updating it - it will be updated to the first address for which socket creation succeeded
         */
        int _createBoundSocket(struct addrinfo*& currentAddr, struct addrinfo* currentBindAddr, const socket_config_handlers_t& configHandlers);
        /**
         * @brief Convert the address, protocol, and service/port to a string for error messages
         *
         * @return std::string String in the format "address:port (protocol)"
         */
        std::string _getFullAddressString() const;

        address_t _address{};
        address_t _bindAddress{};
        socket_protocol _protocol{};

        PtrWrapper<struct addrinfo> _destinationAddrinfo{nullptr, &freeaddrinfo};
        PtrWrapper<struct addrinfo> _bindAddrinfo{nullptr, &freeaddrinfo};

        FdWrapper _socket;
    };
}
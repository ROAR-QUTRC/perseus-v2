#pragma once

#include <fd_wrapper.hpp>
#include <hi_can.hpp>
#include <string>

namespace hi_can
{
    class RawCanInterface : public FilteredCanInterface
    {
    public:
        // note: Providing a default parameter allows the move constructor to work without an explicit default constructor
        /// @brief Instantiates a new RawCanInterface
        /// @param interface_name The interface name (can0, vcan1, etc) to use
        RawCanInterface(const std::string& interface_name = "any");
        // copy constructor
        RawCanInterface(const RawCanInterface& other);
        // move constructor
        RawCanInterface(RawCanInterface&& other) noexcept;
        // copy assignment
        RawCanInterface& operator=(RawCanInterface other);

        friend void swap(RawCanInterface& first, RawCanInterface& second) noexcept
        {
            using std::swap;
            swap(first._filters, second._filters);
            swap(first._interface_name, second._interface_name);
            swap(first._socket, second._socket);
        }

        void transmit(const Packet& packet) override;
        std::optional<Packet> receive(bool blocking = false) override;

        RawCanInterface& add_filter(const addressing::filter_t& address) override;
        RawCanInterface& remove_filter(const addressing::filter_t& address) override;

    private:
        static int _create_socket();
        void _configure_socket(const int& socket);
        void _update_filters();

        // note: NOT const to allow for copy-and-swap
        std::string _interface_name{};
        FdWrapper _socket;
    };
}
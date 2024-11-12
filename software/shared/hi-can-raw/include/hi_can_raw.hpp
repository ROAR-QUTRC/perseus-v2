#pragma once

#include <fd_wrapper.hpp>
#include <hi_can.hpp>
#include <string>

namespace hi_can
{
    class RawCanInterface : public FilteredCanInterface
    {
    public:
        /// @brief Instantiates a new RawCanInterface
        /// @param interfaceName The interface name (can0, vcan1, etc) to use
        RawCanInterface(const std::string& interfaceName);
        // we need the destructor, so we should probably implement the Rule of 5
        ~RawCanInterface();
        // copy constructor
        RawCanInterface(const RawCanInterface& other);
        // move constructor
        RawCanInterface(RawCanInterface&& other) noexcept;
        // copy assignment
        RawCanInterface& operator=(RawCanInterface other);
        // move assignment
        RawCanInterface& operator=(RawCanInterface&& other) noexcept;

        friend void swap(RawCanInterface& first, RawCanInterface& second) noexcept
        {
            using std::swap;
            swap(first._filters, second._filters);
            swap(first._interfaceName, second._interfaceName);
            swap(first._socket, second._socket);
        }

        void transmit(const Packet& packet) override;
        std::optional<Packet> receive(bool blocking = false) override;

    private:
        static int _createSocket();
        void _configureSocket(const int& socket);

        // we only allow a default constructor for the move constructor, so make it private
        RawCanInterface() = default;

        // note: NOT const to allow for copy-and-swap
        std::string _interfaceName{};
        FdWrapper _socket;
    };
}
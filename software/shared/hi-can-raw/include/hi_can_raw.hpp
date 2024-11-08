#pragma once

#include <hi_can_lib.hpp>
#include <string>

namespace hi_can
{
    class RawCanInterface : public FilteredCanInterface
    {
    public:
        /// @brief Instantiates a new RawCanInterface
        /// @param interfaceName The interface name (can0, vcan1, etc) to use
        RawCanInterface(std::string interfaceName);
        ~RawCanInterface();
        // copy constructor
        RawCanInterface(const RawCanInterface& other);

        void transmit(const Packet& packet) const override;
        std::optional<Packet> receive() override;
        void setReceiveCallback(const std::function<void(const Packet&)>& callback) override;

    private:
        const std::string _interfaceName{};

        int _socket = -1;
    };
}
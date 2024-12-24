#pragma once

#include <format>
#include <string>

namespace networking
{
    struct address_t
    {
        std::string hostname{};
        std::string service{};

        explicit operator std::string() const
        {
            if (service == "")
                return hostname;
            return std::format("{}:{}", hostname, service);
        }
    };
    enum class socket_protocol
    {
        TCP,
        UDP
    };
    std::string getStringFromProtocol(const socket_protocol& protocol)
    {
        return (protocol == socket_protocol::TCP) ? "TCP" : "UDP";
    }
}
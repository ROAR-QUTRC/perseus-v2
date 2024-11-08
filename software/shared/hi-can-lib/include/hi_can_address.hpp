#pragma once

#include <cstdint>

namespace hi_can
{
    typedef uint32_t can_address_t;

    const size_t MAX_PACKET_LEN = 8;
    const can_address_t MAX_ADDRESS = 0x1FFFFFFF;

    const uint8_t CANLIB_SYSTEM_ADDRESS_BITS = 5;
    const uint8_t CANLIB_SUBSYSTEM_ADDRESS_BITS = 4;
    const uint8_t CANLIB_DEVICE_ADDRESS_BITS = 8;
    const uint8_t CANLIB_GROUP_ADDRESS_BITS = 8;
    const uint8_t CANLIB_PARAM_ADDRESS_BITS = 4;

    const uint8_t CANLIB_PARAM_ADDRESS_POS = 0;
    const uint8_t CANLIB_GROUP_ADDRESS_POS = (CANLIB_PARAM_ADDRESS_POS + CANLIB_PARAM_ADDRESS_BITS);
    const uint8_t CANLIB_DEVICE_ADDRESS_POS = (CANLIB_GROUP_ADDRESS_POS + CANLIB_GROUP_ADDRESS_BITS);
    const uint8_t CANLIB_SUBSYSTEM_ADDRESS_POS = (CANLIB_DEVICE_ADDRESS_POS + CANLIB_DEVICE_ADDRESS_BITS);
    const uint8_t CANLIB_SYSTEM_ADDRESS_POS = (CANLIB_SUBSYSTEM_ADDRESS_POS + CANLIB_SUBSYSTEM_ADDRESS_BITS);

    struct vesc_address_t
    {
        uint8_t commandId = 0;
        uint8_t vescId = 0;

        operator can_address_t() const
        {
            return (static_cast<uint32_t>(commandId) << 8) | static_cast<uint32_t>(vescId);
        }
    };
    struct hi_can_address_t
    {
        uint8_t system : CANLIB_SYSTEM_ADDRESS_BITS;
        uint8_t subsystem : CANLIB_SUBSYSTEM_ADDRESS_BITS;
        uint8_t device : CANLIB_DEVICE_ADDRESS_BITS;
        uint8_t group : CANLIB_GROUP_ADDRESS_BITS;
        uint8_t parameter : CANLIB_PARAM_ADDRESS_BITS;

        operator can_address_t() const
        {
            return (static_cast<uint32_t>(system) << CANLIB_SYSTEM_ADDRESS_POS) |
                   (static_cast<uint32_t>(subsystem) << CANLIB_SUBSYSTEM_ADDRESS_POS) |
                   (static_cast<uint32_t>(device) << CANLIB_DEVICE_ADDRESS_POS) |
                   (static_cast<uint32_t>(group) << CANLIB_GROUP_ADDRESS_POS) |
                   (static_cast<uint32_t>(parameter) << CANLIB_PARAM_ADDRESS_POS);
        }
    };

    namespace addresses
    {
        namespace drive
        {
        }
    }
}
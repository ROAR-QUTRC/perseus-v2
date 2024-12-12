#include "hi_can_address.hpp"

using namespace hi_can;
using namespace addressing;

namespace hi_can::addressing::legacy
{

    address_t power::control::power_bus::getAddress(group id, uint8_t parameter)
    {
        return address_t{SYSTEM_ID,
                         SUBSYSTEM_ID,
                         static_cast<uint8_t>(device::ROVER_CONTROL_BOARD),
                         static_cast<uint8_t>(id),
                         parameter};
    }
}
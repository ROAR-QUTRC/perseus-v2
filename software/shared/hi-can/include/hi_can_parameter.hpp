#include <cstdint>

#pragma pack(push, 1)
namespace hi_can::parameter
{
    namespace drive
    {
        namespace vesc
        {
            struct type_1_status
            {
            };
        }
    }
    namespace power
    {
        namespace bms
        {
            struct pack_status
            {
            };
        }
        namespace distribution
        {
            struct bus_status_t
            {
                enum class status : uint8_t
                {
                    OFF = 0,
                    ON,
                    PRECHARGING,
                    PRECHARGE_FAIL,
                    SWITCH_FAILED,
                    OVERLOAD,
                    FAULT,
                };
                status status;
                uint16_t voltage;  // in mV
                uint32_t current;  // in mA
            };
        }
    }
    namespace excavation
    {

    }
}
#pragma pack(pop)
#include <cstdint>

#include "hi_can.hpp"
#include "hi_can_address.hpp"

#pragma pack(push, 1)
namespace hi_can::parameter
{
    class ParameterGroup
    {
    public:
        ParameterGroup(addressing::raw_address_t deviceAddress, PacketManager& packetManager)
            : _deviceAddress(deviceAddress), _packetManager(packetManager) {}
        ~ParameterGroup()
        {
            deregisterParameters();
        }

        // no copy or move constructors or operators - doesn't make sense
        ParameterGroup(const ParameterGroup&) = delete;
        ParameterGroup(ParameterGroup&&) = delete;
        ParameterGroup& operator=(const ParameterGroup&) = delete;
        ParameterGroup& operator=(ParameterGroup&&) = delete;

        virtual void registerParameters() = 0;
        virtual void deregisterParameters() = 0;

    protected:
        addressing::raw_address_t _deviceAddress;
        PacketManager& _packetManager;
    };
    namespace drive
    {
        namespace vesc
        {
            class VescParameterGroup : public ParameterGroup
            {
                void registerParameters() override;
                void deregisterParameters() override;
            };
            struct type_1_status
            {
            };
            struct type_2_status
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
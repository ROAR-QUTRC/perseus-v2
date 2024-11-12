#pragma once

#include <cstddef>
#include <cstdint>

namespace hi_can
{
    namespace address
    {

        typedef uint32_t raw_address_t;
        typedef uint32_t raw_mask_t;

        constexpr uint8_t ADDRESS_LENGTH = 29;
        constexpr size_t MAX_PACKET_LEN = 8;
        constexpr raw_address_t MAX_ADDRESS = 0x1FFFFFFFUL;
        constexpr raw_mask_t MASK_ALL = 0x1FFFFFFFUL;
        constexpr raw_mask_t MASK_NONE = 0x00000000UL;

        constexpr uint8_t SYSTEM_ADDRESS_BITS = 5;
        constexpr uint8_t SUBSYSTEM_ADDRESS_BITS = 4;
        constexpr uint8_t DEVICE_ADDRESS_BITS = 8;
        constexpr uint8_t GROUP_ADDRESS_BITS = 8;
        constexpr uint8_t PARAM_ADDRESS_BITS = 4;

        constexpr uint8_t PARAM_ADDRESS_POS = 0;
        constexpr uint8_t GROUP_ADDRESS_POS = (PARAM_ADDRESS_POS + PARAM_ADDRESS_BITS);
        constexpr uint8_t DEVICE_ADDRESS_POS = (GROUP_ADDRESS_POS + GROUP_ADDRESS_BITS);
        constexpr uint8_t SUBSYSTEM_ADDRESS_POS = (DEVICE_ADDRESS_POS + DEVICE_ADDRESS_BITS);
        constexpr uint8_t SYSTEM_ADDRESS_POS = (SUBSYSTEM_ADDRESS_POS + SUBSYSTEM_ADDRESS_BITS);

        constexpr raw_mask_t SYSTEM_MASK = (MASK_ALL << SYSTEM_ADDRESS_POS) & MASK_ALL;
        constexpr raw_mask_t SUBSYSTEM_MASK = (MASK_ALL << SUBSYSTEM_ADDRESS_POS) & MASK_ALL;
        constexpr raw_mask_t DEVICE_MASK = (MASK_ALL << DEVICE_ADDRESS_POS) & MASK_ALL;
        constexpr raw_mask_t GROUP_MASK = (MASK_ALL << GROUP_ADDRESS_POS) & MASK_ALL;
        constexpr raw_mask_t PARAM_MASK = (MASK_ALL << PARAM_ADDRESS_POS) & MASK_ALL;

        /// @brief A CAN address and mask for filtering
        struct filter_t
        {
            /// @brief The address to accept
            raw_address_t address = MAX_ADDRESS;
            /// @brief The mask of address bits to care about
            raw_mask_t mask = MASK_ALL;

            // need to define the comparison operators for std::set
            // more specific filters (greater masks) should be sorted first, and as such compare as less than
            // If the masks are the same, sort by address
            constexpr bool operator<(const filter_t& other) const
            {
                if (mask != other.mask)
                    return mask > other.mask;
                return address < other.address;
            }
            constexpr bool operator>(const filter_t& other) const
            {
                if (mask != other.mask)
                    return mask < other.mask;
                return address > other.address;
            }
            constexpr bool operator==(const filter_t& other) const
            {
                return address == other.address && mask == other.mask;
            }
        };

        struct structured_address_t
        {
            virtual constexpr operator raw_address_t() const = 0;
        };
        struct standard_address_t : public structured_address_t
        {
            uint8_t system : SYSTEM_ADDRESS_BITS;
            uint8_t subsystem : SUBSYSTEM_ADDRESS_BITS;
            uint8_t device : DEVICE_ADDRESS_BITS;
            uint8_t group : GROUP_ADDRESS_BITS;
            uint8_t parameter : PARAM_ADDRESS_BITS;
            const uint8_t _padding : (32 -
                                      SYSTEM_ADDRESS_BITS -
                                      SUBSYSTEM_ADDRESS_BITS -
                                      DEVICE_ADDRESS_BITS -
                                      GROUP_ADDRESS_BITS -
                                      PARAM_ADDRESS_BITS) = 0;

            constexpr operator raw_address_t() const override
            {
                return (static_cast<uint32_t>(system) << SYSTEM_ADDRESS_POS) |
                       (static_cast<uint32_t>(subsystem) << SUBSYSTEM_ADDRESS_POS) |
                       (static_cast<uint32_t>(device) << DEVICE_ADDRESS_POS) |
                       (static_cast<uint32_t>(group) << GROUP_ADDRESS_POS) |
                       (static_cast<uint32_t>(parameter) << PARAM_ADDRESS_POS);
            }
        };

        namespace drive
        {
            constexpr uint8_t SYSTEM_ID = 0x00;
            namespace vesc
            {
                constexpr uint8_t SUBSYSTEM_ID = 0x00;
                enum class device
                {
                    FRONT_LEFT = 0,
                    FRONT_RIGHT = 1,
                    REAR_LEFT = 2,
                    REAR_RIGHT = 3,
                };
                enum class command_id
                {
                    SET_DUTY = 0,
                    SET_CURRENT = 1,
                    SET_CURRENT_BRAKE = 2,
                    SET_RPM = 3,
                    SET_POS = 4,
                    SET_CURRENT_REL = 10,
                    SET_CURRENT_BRAKE_REL = 11,
                    SET_CURRENT_HANDBRAKE = 12,
                    SET_CURRENT_HANDBRAKE_REL = 13,
                };

                struct address_t : public structured_address_t
                {
                    command_id command = command_id::SET_DUTY;
                    device vesc = device::FRONT_LEFT;

                    constexpr operator raw_address_t() const override
                    {
                        return (static_cast<uint32_t>(command) << 8) | static_cast<uint32_t>(vesc);
                    }
                };
            }
        }
        namespace power
        {
            constexpr uint8_t SYSTEM_ID = 0x01;
            namespace battery
            {
                constexpr uint8_t SUBSYSTEM_ID = 0x00;
                enum class device
                {
                    BATTERY_1 = 0,
                    BATTERY_2 = 1,
                    BATTERY_3 = 2,
                    BATTERY_4 = 3,
                    BATTERY_5 = 4,
                    BATTERY_6 = 5,
                    BATTERY_7 = 6,
                    BATTERY_8 = 7,
                };
            }
            namespace distribution
            {
                constexpr uint8_t SUBSYSTEM_ID = 0x01;
                enum class device
                {
                    ROVER_CONTROL_BOARD = 0,
                };
            }
        }
        namespace compute
        {
            constexpr uint8_t SYSTEM_ID = 0x02;
            namespace primary
            {
                constexpr uint8_t SUBSYSTEM_ID = 0x00;
                enum class device
                {
                    BIG_BRAIN = 0,
                    MEDIUM_BRAIN = 1,
                };
            }
        }
        namespace post_landing
        {
            constexpr uint8_t SYSTEM_ID = 0x03;
        }
        namespace excavation
        {
            constexpr uint8_t SYSTEM_ID = 0x04;
            namespace arm
            {
                constexpr uint8_t SUBSYSTEM_ID = 0x00;
            }
        }
        namespace space_resources
        {
            constexpr uint8_t SYSTEM_ID = 0x05;
        }
    }
}
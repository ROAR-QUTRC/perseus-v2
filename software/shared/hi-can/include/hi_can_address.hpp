#pragma once

#include <cstddef>
#include <cstdint>

namespace hi_can
{
    /// @brief Contains all the types, classes, constants, and functions for
    ///        working with CAN addresses, as well as all of the addresses of the devices currently on the bus
    namespace addressing
    {
        // these are typedef'd to make it clear in which contexts they're being used
        /// @brief A raw CAN address
        typedef uint32_t raw_address_t;
        /// @brief A CAN address mask
        typedef uint32_t mask_t;

        constexpr uint8_t ADDRESS_LENGTH = 29;
        constexpr size_t MAX_PACKET_LEN = 8;
        constexpr raw_address_t MAX_ADDRESS = 0x1FFFFFFFUL;
        constexpr raw_address_t MAX_SHORT_ADDRESS = 0x7FFUL;
        constexpr mask_t MASK_ALL = 0x1FFFFFFFUL;
        constexpr mask_t MASK_NONE = 0x00000000UL;

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

        constexpr mask_t SYSTEM_MASK = (MASK_ALL << SYSTEM_ADDRESS_POS) & MASK_ALL;
        constexpr mask_t SUBSYSTEM_MASK = (MASK_ALL << SUBSYSTEM_ADDRESS_POS) & MASK_ALL;
        constexpr mask_t DEVICE_MASK = (MASK_ALL << DEVICE_ADDRESS_POS) & MASK_ALL;
        constexpr mask_t GROUP_MASK = (MASK_ALL << GROUP_ADDRESS_POS) & MASK_ALL;
        constexpr mask_t PARAM_MASK = (MASK_ALL << PARAM_ADDRESS_POS) & MASK_ALL;

        /// @brief A CAN address with flags for RTR, error, and extended (29-bit) addressing
        struct flagged_address_t
        {
            raw_address_t address = 0;
            bool rtr = false;
            bool error = false;
            bool extended = true;

            constexpr flagged_address_t() = default;
            constexpr flagged_address_t(raw_address_t address, bool rtr = false, bool error = false, bool extended = true)
                : address(address & (extended ? MAX_ADDRESS : MAX_SHORT_ADDRESS)),
                  rtr(rtr),
                  error(error),
                  extended(extended) {}
            constexpr explicit operator raw_address_t() const
            {
                return address & (extended ? MAX_ADDRESS : MAX_SHORT_ADDRESS);
            }

            constexpr auto operator<=>(const flagged_address_t& other) const
            {
                if (address != other.address)
                    return address <=> other.address;
                if (rtr != other.rtr)
                    return rtr <=> other.rtr;
                if (error != other.error)
                    return error <=> other.error;
                return extended <=> other.extended;
            }
            constexpr auto operator==(const flagged_address_t& other) const
            {
                return (address == other.address) && (rtr == other.rtr) && (error == other.error) && (extended == other.extended);
            }
            constexpr auto operator!=(const flagged_address_t& other) const
            {
                return !(*this == other);
            }
        };

        /// @brief A CAN address and mask for filtering, as well as whether to match RTR and error frames
        struct filter_t
        {
            /// @brief The address to accept
            flagged_address_t address = MAX_ADDRESS;
            /// @brief The mask of address bits to care about
            mask_t mask = MASK_ALL;
            bool matchRtr = false;
            bool matchError = true;

            // need to define the comparison operators for std::set
            /// @brief Compare two filters for sorting
            /// @param other Filter to compare against
            /// @return The result of the comparison
            ///
            /// Compares by, in order:
            /// - mask
            /// - address
            /// - matchRtr
            /// - matchError
            /// More specific filters (greater masks) should be sorted first, and as such compare as less than, by flipping the comparison.
            constexpr auto operator<=>(const filter_t& other) const
            {
                if (mask != other.mask)
                    return other.mask <=> mask;
                if (address != other.address)
                    return address <=> other.address;
                if (matchRtr != other.matchRtr)
                    return matchRtr <=> other.matchRtr;
                return matchError <=> other.matchError;
            }

            /// @brief Check if address matches the filter
            /// @param address Address to check
            /// @return Whether the address matches the filter
            constexpr bool matches(const flagged_address_t& address) const
            {
                return (static_cast<raw_address_t>(address) & mask) == (static_cast<raw_address_t>(this->address) & mask) &&
                       (this->address.extended == address.extended) &&
                       (!matchRtr || (this->address.rtr == (address.rtr))) &&
                       (!matchError || (this->address.error == (address.error)));
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
                return (static_cast<raw_address_t>(system) << SYSTEM_ADDRESS_POS) |
                       (static_cast<raw_address_t>(subsystem) << SUBSYSTEM_ADDRESS_POS) |
                       (static_cast<raw_address_t>(device) << DEVICE_ADDRESS_POS) |
                       (static_cast<raw_address_t>(group) << GROUP_ADDRESS_POS) |
                       (static_cast<raw_address_t>(parameter) << PARAM_ADDRESS_POS);
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
                        return (static_cast<raw_address_t>(command) << 8) | static_cast<raw_address_t>(vesc);
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
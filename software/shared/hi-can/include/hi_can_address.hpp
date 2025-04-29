#pragma once

#include <compare>
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
        /// @brief A raw CAN address mask
        typedef uint32_t mask_t;

        /// @brief The maximum length of a CAN address in bits
        constexpr uint8_t ADDRESS_LENGTH = 29;
        /// @brief The maximum length of a CAN packet in bytes
        constexpr size_t MAX_PACKET_LEN = 8;
        /// @brief The maximum value of a CAN address
        constexpr raw_address_t MAX_ADDRESS = 0x1FFFFFFFUL;
        /// @brief The maximum value of a short (non-extended) CAN address
        constexpr raw_address_t MAX_SHORT_ADDRESS = 0x7FFUL;
        /// @brief Mask for all bits in a CAN address
        constexpr mask_t MASK_ALL = 0x1FFFFFFFUL;
        /// @brief Mask for no bits of the CAN address
        constexpr mask_t MASK_NONE = 0x00000000UL;

        /// @brief The number of bits in a @ref raw_address_t dedicated to identifying the system
        constexpr uint8_t SYSTEM_ADDRESS_BITS = 6;
        /// @brief The number of bits in a @ref raw_address_t dedicated to identifying the subsystem
        constexpr uint8_t SUBSYSTEM_ADDRESS_BITS = 3;
        /// @brief The number of bits in a @ref raw_address_t dedicated to identifying the specific device
        constexpr uint8_t DEVICE_ADDRESS_BITS = 4;
        /// @brief The number of bits in a @ref raw_address_t dedicated to identifying the parameter group
        constexpr uint8_t GROUP_ADDRESS_BITS = 8;
        /// @brief The number of bits in a @ref raw_address_t dedicated to identifying the parameter in the group
        constexpr uint8_t PARAM_ADDRESS_BITS = 8;

        /// @brief Offset bit count of the parameter ID in a @ref raw_address_t
        constexpr uint8_t PARAM_ADDRESS_POS = 0;
        /// @brief Offset bit count of the parameter group ID in a @ref raw_address_t
        constexpr uint8_t GROUP_ADDRESS_POS = (PARAM_ADDRESS_POS + PARAM_ADDRESS_BITS);
        /// @brief Offset bit count of the device ID in a @ref raw_address_t
        constexpr uint8_t DEVICE_ADDRESS_POS = (GROUP_ADDRESS_POS + GROUP_ADDRESS_BITS);
        /// @brief Offset bit count of the subsystem ID in a @ref raw_address_t
        constexpr uint8_t SUBSYSTEM_ADDRESS_POS = (DEVICE_ADDRESS_POS + DEVICE_ADDRESS_BITS);
        /// @brief Offset bit count of the system ID in a @ref raw_address_t
        constexpr uint8_t SYSTEM_ADDRESS_POS = (SUBSYSTEM_ADDRESS_POS + SUBSYSTEM_ADDRESS_BITS);

        /// @brief Mask for the system ID in a @ref raw_address_t
        constexpr mask_t SYSTEM_MASK = (MASK_ALL << SYSTEM_ADDRESS_POS) & MASK_ALL;
        /// @brief Mask for the subsystem ID in a @ref raw_address_t
        constexpr mask_t SUBSYSTEM_MASK = (MASK_ALL << SUBSYSTEM_ADDRESS_POS) & MASK_ALL;
        /// @brief Mask for the device ID in a @ref raw_address_t
        constexpr mask_t DEVICE_MASK = (MASK_ALL << DEVICE_ADDRESS_POS) & MASK_ALL;
        /// @brief Mask for the parameter group ID in a @ref raw_address_t
        constexpr mask_t GROUP_MASK = (MASK_ALL << GROUP_ADDRESS_POS) & MASK_ALL;
        /// @brief Mask for the parameter ID in a @ref raw_address_t
        constexpr mask_t PARAM_MASK = (MASK_ALL << PARAM_ADDRESS_POS) & MASK_ALL;

        /// @brief An address which can be converted to a raw address - all addressing in the Hi-CAN library should inherit from this if possible
        struct structured_address_t
        {
            structured_address_t() = default;
            /// @brief Convert to a raw address
            /// @return The raw address
            virtual constexpr operator raw_address_t() const = 0;
        };
        /// @brief A CAN address with flags for RTR, error, and extended (29-bit) addressing
        struct flagged_address_t : public structured_address_t
        {
            /// @brief The base raw address
            raw_address_t address = 0;
            /// @brief Whether the address is an RTR frame
            bool isRtr = false;
            /// @brief Whether the address is an error frame
            bool isError = false;
            /// @brief Whether the address is an extended (29-bit) address
            bool isExtended = true;

            /// @brief Default constructor - zero address, not RTR or error, extended
            constexpr flagged_address_t() = default;
            /// @brief Construct a flagged address from a raw address and flags
            constexpr flagged_address_t(raw_address_t address, bool isRtr = false, bool isError = false, bool isExtended = true)
                : address(address & (isExtended ? MAX_ADDRESS : MAX_SHORT_ADDRESS)),
                  isRtr(isRtr),
                  isError(isError),
                  isExtended(isExtended)
            {
            }

            /// @brief Convert a flagged address to a raw address
            constexpr explicit operator raw_address_t() const override
            {
                return address & (isExtended ? MAX_ADDRESS : MAX_SHORT_ADDRESS);
            }

            /// Sorts first by address, then @ref isRtr flag, then @ref isError flag, then @ref isExtended flag
            /// @brief Compare two flagged addresses (for sorting)
            /// @param other Address to compare against
            /// @return The result of the comparison
            constexpr auto operator<=>(const flagged_address_t& other) const
            {
                if (address != other.address)
                    return address <=> other.address;
                if (isRtr != other.isRtr)
                    return isRtr <=> other.isRtr;
                if (isError != other.isError)
                    return isError <=> other.isError;
                return isExtended <=> other.isExtended;
            }
            /// @brief Check if two addresses are equal. Checks address and all flags.
            /// @param other Address to compare against
            /// @return Whether the addresses are equal
            constexpr auto operator==(const flagged_address_t& other) const
            {
                return (address == other.address) && (isRtr == other.isRtr) && (isError == other.isError) && (isExtended == other.isExtended);
            }
            /// @brief Check if two addresses are not equal
            /// @param other Address to compare against
            /// @return Whether the addresses differ
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
            /// @brief Whether to match the RTR flag as well
            bool shouldMatchRtr = false;
            /// @brief Whether to match the isError flag as well
            bool shouldMatchError = true;

            // need to define the comparison operators for std::set
            /// Compares by, in order:
            /// - mask
            /// - address
            /// - shouldMatchRtr
            /// - shouldMatchError
            /// More specific filters (greater masks) should be sorted first, and as such compare as less than, by flipping the comparison.
            /// @brief Compare two filters for sorting
            /// @param other Filter to compare against
            /// @return The result of the comparison
            constexpr auto operator<=>(const filter_t& other) const
            {
                if (mask != other.mask)
                    return other.mask <=> mask;
                if (address != other.address)
                    return address <=> other.address;
                if (shouldMatchRtr != other.shouldMatchRtr)
                    return shouldMatchRtr <=> other.shouldMatchRtr;
                return shouldMatchError <=> other.shouldMatchError;
            }

            /// @brief Check if address matches the filter
            /// @param address Address to check
            /// @return Whether the address matches the filter
            constexpr bool matches(const flagged_address_t& address) const
            {
                return (static_cast<raw_address_t>(address) & mask) == (static_cast<raw_address_t>(this->address) & mask) &&
                       (this->address.isExtended == address.isExtended) &&
                       (!shouldMatchRtr || (this->address.isRtr == (address.isRtr))) &&
                       (!shouldMatchError || (this->address.isError == (address.isError)));
            }

            constexpr raw_address_t getMaskedAddress() const
            {
                return static_cast<raw_address_t>(address) & mask;
            }
            constexpr raw_address_t getUnmaskedAddress() const
            {
                return static_cast<raw_address_t>(address) & (~mask);
            }
        };

        /// @brief A standard address for a Hi-CAN compliant device
        struct standard_address_t : public structured_address_t
        {
            /// @brief The system ID
            uint8_t system : SYSTEM_ADDRESS_BITS;
            /// @brief The subsystem ID
            uint8_t subsystem : SUBSYSTEM_ADDRESS_BITS;
            /// @brief The device ID
            uint8_t device : DEVICE_ADDRESS_BITS;
            /// @brief The parameter group ID
            uint8_t group : GROUP_ADDRESS_BITS;
            /// @brief The parameter ID
            uint8_t parameter : PARAM_ADDRESS_BITS;
            /// @brief Padding to fill out the rest of 32 bits so it's aligned
            const uint8_t _padding : (32 -
                                      SYSTEM_ADDRESS_BITS -
                                      SUBSYSTEM_ADDRESS_BITS -
                                      DEVICE_ADDRESS_BITS -
                                      GROUP_ADDRESS_BITS -
                                      PARAM_ADDRESS_BITS) = 0;

            standard_address_t(const uint8_t& system = 0x00,
                               const uint8_t& subsystem = 0x00,
                               const uint8_t& device = 0x00,
                               const uint8_t& group = 0x00,
                               const uint8_t& parameter = 0x00)
                : system(system),
                  subsystem(subsystem),
                  device(device),
                  group(group),
                  parameter(parameter)
            {
            }
            standard_address_t(const standard_address_t& deviceAddress, const uint8_t& group, const uint8_t& parameter)
                : system(deviceAddress.system),
                  subsystem(deviceAddress.subsystem),
                  device(deviceAddress.device),
                  group(group),
                  parameter(parameter)
            {
            }

            constexpr operator raw_address_t() const override
            {
                return (static_cast<raw_address_t>(system) << SYSTEM_ADDRESS_POS) |
                       (static_cast<raw_address_t>(subsystem) << SUBSYSTEM_ADDRESS_POS) |
                       (static_cast<raw_address_t>(device) << DEVICE_ADDRESS_POS) |
                       (static_cast<raw_address_t>(group) << GROUP_ADDRESS_POS) |
                       (static_cast<raw_address_t>(parameter) << PARAM_ADDRESS_POS);
            }
        };

        /// @brief Namespace containing all addresses in the drive system
        namespace drive
        {
            /// @brief The drive system ID
            constexpr uint8_t SYSTEM_ID = 0x00;
            /// Note that VESCs kinda have to be shoehorned into the Hi-CAN spec,
            /// so their address isn't *entirely* compliant with the standard.
            /// By treating them as "system 0, subsystem 0" devices, we can isolate the issues
            /// and make it mostly work
            /// @brief Namespace containing all addresses in the VESC subsystem
            namespace vesc
            {
                /// @brief The VESC subsystem ID
                constexpr uint8_t SUBSYSTEM_ID = 0x00;
                /// @brief The main drive VESC device IDs
                enum class device
                {
                    FRONT_LEFT = 0,
                    FRONT_RIGHT = 1,
                    REAR_LEFT = 2,
                    REAR_RIGHT = 3,
                };
                /// @brief VESC command IDs
                enum class command_id
                {
                    SET_DUTY = 0,
                    SET_CURRENT = 1,
                    SET_CURRENT_BRAKE = 2,
                    SET_RPM = 3,
                    SET_POS = 4,
                    STATUS_1 = 9,
                    SET_CURRENT_REL = 10,
                    SET_CURRENT_BRAKE_REL = 11,
                    SET_CURRENT_HANDBRAKE = 12,
                    SET_CURRENT_HANDBRAKE_REL = 13,
                    STATUS_2 = 14,
                    STATUS_3 = 15,
                    STATUS_4 = 16,
                    STATUS_5 = 27,
                    STATUS_6 = 58,
                };

                /// @brief VESC command packet address
                struct address_t : public structured_address_t
                {
                    address_t(const uint8_t& vesc, const command_id& command)
                        : vesc(vesc),
                          command(command)
                    {
                    }
                    /// @brief The VESC device ID
                    uint8_t vesc = static_cast<uint8_t>(device::FRONT_LEFT);
                    /// @brief The VESC command ID
                    command_id command = command_id::SET_DUTY;

                    constexpr operator raw_address_t() const override
                    {
                        return (static_cast<raw_address_t>(command) << 8) | static_cast<raw_address_t>(vesc);
                    }
                };
            }
        }
        /// @brief Namespace containing all addresses in the power system
        namespace power
        {
            /// @brief The power system ID
            constexpr uint8_t SYSTEM_ID = 0x01;
            /// @brief Namespace containing all addresses in the battery subsystem
            namespace battery
            {
                /// @brief The battery subsystem ID
                constexpr uint8_t SUBSYSTEM_ID = 0x00;
                /// @brief List of battery device IDs
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
            /// @brief Namespace containing all addresses in the power distribution subsystem
            namespace distribution
            {
                /// @brief The power distribution subsystem ID
                constexpr uint8_t SUBSYSTEM_ID = 0x01;
                /// @brief List of power distribution device IDs
                enum class device
                {
                    ROVER_CONTROL_BOARD = 0,
                };
            }
        }
        /// @brief Namespace containing all addresses in the compute system
        namespace compute
        {
            /// @brief The compute system ID
            constexpr uint8_t SYSTEM_ID = 0x02;
            /// @brief Namespace containing all addresses in the primary compute subsystem
            namespace primary
            {
                /// @brief The primary compute subsystem ID
                constexpr uint8_t SUBSYSTEM_ID = 0x00;
                /// @brief List of primary compute device IDs
                enum class device
                {
                    BIG_BRAIN = 0,
                    MEDIUM_BRAIN = 1,
                };
            }
        }

        // payload systems go below here
        /// @brief Namespace containing all addresses in the post-landing system
        namespace post_landing
        {
            /// @brief The post-landing system ID
            constexpr uint8_t SYSTEM_ID = 0x03;
        }
        /// @brief Namespace containing all addresses in the excavation system
        namespace excavation
        {
            /// @brief The excavation system ID
            constexpr uint8_t SYSTEM_ID = 0x04;
            /// @brief Namespace containing all addresses in the arm subsystem
            namespace arm
            {
                /// @brief The arm subsystem ID
                constexpr uint8_t SUBSYSTEM_ID = 0x00;
            }
        }
        /// @brief Namespace containing all addresses in the space resources system
        namespace space_resources
        {
            /// @brief The space resources system ID
            constexpr uint8_t SYSTEM_ID = 0x05;
        }
        namespace shared
        {
            /// @brief System ID for devices not specific to one system only
            constexpr uint8_t SYSTEM_ID = 0x06;
            namespace elevator
            {
                /// @brief The elevator platform subsystem ID
                constexpr uint8_t SUBSYSTEM_ID = 0x00;
                namespace elevator
                {
                    const standard_address_t DEVICE_ADDRESS{SYSTEM_ID, SUBSYSTEM_ID, 0x00};
                    namespace motor
                    {
                        constexpr uint8_t GROUP_ID = 0x01;
                        enum class parameter
                        {
                            SPEED = 0x00,
                        };
                    }
                };
            }
        }
        // legacy addresses for old hardware
        /// @brief Namespace containing all addresses in the legacy system
        namespace legacy
        {
            struct address_t : public structured_address_t
            {
                /// @brief The system ID
                uint8_t system : 5;
                /// @brief The subsystem ID
                uint8_t subsystem : 4;
                /// @brief The device ID
                uint8_t device : 8;
                /// @brief The parameter group ID
                uint8_t group : 8;
                /// @brief The parameter ID
                uint8_t parameter : 4;
                /// @brief Padding to fill out the rest of 32 bits so it's aligned
                const uint8_t _padding : (32 - 5 - 4 - 8 - 8 - 4) = 0;

                address_t(const uint8_t& system = 0x00,
                          const uint8_t& subsystem = 0x00,
                          const uint8_t& device = 0x00,
                          const uint8_t& group = 0x00,
                          const uint8_t& parameter = 0x00)
                    : system(system),
                      subsystem(subsystem),
                      device(device),
                      group(group),
                      parameter(parameter)
                {
                }
                address_t(address_t deviceAddress, const uint8_t& group, const uint8_t& parameter)
                    : system(deviceAddress.system),
                      subsystem(deviceAddress.subsystem),
                      device(deviceAddress.device),
                      group(group),
                      parameter(parameter)
                {
                }

                constexpr operator raw_address_t() const override
                {
                    return (static_cast<raw_address_t>(system) << 24) |
                           (static_cast<raw_address_t>(subsystem) << 20) |
                           (static_cast<raw_address_t>(device) << 12) |
                           (static_cast<raw_address_t>(group) << 4) |
                           (static_cast<raw_address_t>(parameter) << 0);
                }
            };
            // SYSTEMS
            namespace power
            {
                constexpr uint8_t SYSTEM_ID = 0x01;
                namespace control
                {
                    constexpr uint8_t SUBSYSTEM_ID = 0x00;
                    enum class device
                    {
                        ROVER_CONTROL_BOARD = 0x00,
                    };
                    // DEVICES
                    namespace rcb
                    {
                        enum class groups
                        {
                            CONTACTOR = 0x01,
                            COMPUTE_BUS = 0x02,
                            DRIVE_BUS = 0x03,
                            AUX_BUS = 0x04,
                            SPARE_BUS = 0x05,
                        };
                    }
                    // PARAMETER GROUPS
                    namespace contactor
                    {
                        enum class parameter
                        {
                            SHUTDOWN = 0x00
                        };
                    }
                    namespace power_bus
                    {
                        enum class parameter
                        {
                            CONTROL_IMMEDIATE = 0x00,
                            CONTROL_SCHEDULED = 0x01,
                            CURRENT_LIMIT = 0x02,
                            POWER_STATUS = 0x03,
                        };
                    }
                }
            }
            namespace drive
            {
                constexpr uint8_t SYSTEM_ID = 0x02;
                namespace motors
                {
                    constexpr uint8_t SUBSYSTEM_ID = 0x00;
                    enum class device
                    {
                        FRONT_LEFT_MOTOR = 0x00,
                        FRONT_RIGHT_MOTOR = 0x01,
                        REAR_LEFT_MOTOR = 0x02,
                        REAR_RIGHT_MOTOR = 0x03,
                    };
                    // DEVICES
                    namespace mcb
                    {
                        enum class groups
                        {
                            ESC = 0x01,
                        };
                    }
                    // PARAMETER GROUPS
                    namespace esc
                    {
                        constexpr uint8_t esc = 0x01;
                        enum class parameter
                        {
                            SPEED = 0x00,
                            LIMITS = 0x01,
                            STATUS = 0x02,
                            POSITION = 0x03,
                        };
                    }
                }
            }
            namespace excavation
            {
                constexpr uint8_t SYSTEM_ID = 0x03;
                namespace bucket
                {
                    constexpr uint8_t SUBSYSTEM_ID = 0x00;
                    enum class device
                    {
                        BUCKET = 0x00,
                    };
                    namespace bucket
                    {
                        namespace motors
                        {
                            constexpr uint8_t GROUP_ID = 0x01;
                            enum class parameter
                            {
                                LIFT_SPEED = 0x00,
                                TILT_SPEED = 0x01,
                                JAWS_SPEED = 0x02,
                            };
                        }
                        namespace manipulation
                        {
                            constexpr uint8_t GROUP_ID = 0x02;
                            enum class parameter
                            {
                                SPIN_SPEED = 0x00,
                                ELECTROMAGNET = 0x01,
                            };
                        }
                    }
                }
            }
        }
        // PARAMETER GROUPS
        namespace status
        {
            /// @brief Status parameter group ID - will always be 0x00
            constexpr uint8_t GROUP_ID = 0x00;
            /// @brief Status parameter IDs
            enum class parameter
            {
                /// @brief Power control
                POWER = 0x00,
                /// @brief Device current status
                STATUS = 0x01,
                /// @brief Device CPU loading (if applicable)
                CPU = 0x02,
            };
        }
    }
}
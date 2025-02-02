#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <system_error>

namespace st3215
{
    /**
     * @brief Protocol packet header bytes
     */
    static constexpr std::array<uint8_t, 2> PACKET_HEADER{0xFF, 0xFF};

    /**
     * @brief Broadcast ID for commands that should affect all servos
     */
    static constexpr uint8_t BROADCAST_ID = 0xFE;

    /**
     * @brief Maximum number of servos that can be controlled (based on ID range)
     */
    static constexpr uint8_t MAX_SERVOS = 253;  // IDs 0-252, 254 is broadcast

    /**
     * @brief Servo operating modes
     */
    enum class operating_mode_t : uint8_t
    {
        POSITION = 0,   ///< Position control mode (0-1023)
        SPEED = 1,      ///< Speed control mode (-1000 to 1000)
        OPEN_LOOP = 2,  ///< Direct PWM control
        INVALID = 3     ///< Invalid mode
    };

    /**
     * @brief Instruction codes for servo communication
     */
    enum class instruction_t : uint8_t
    {
        PING = 0x01,       ///< Check if servo exists
        READ = 0x02,       ///< Read from memory
        WRITE = 0x03,      ///< Write to memory
        REG_WRITE = 0x04,  ///< Write to register (pending)
        ACTION = 0x05,     ///< Execute pending REG_WRITE
        SYNC_WRITE = 0x83  ///< Write to multiple servos
    };

    /**
     * @brief Error codes that can be returned by the servo
     */
    enum class error_t : uint8_t
    {
        NONE = 0x00,         ///< No error
        VOLTAGE = 0x01,      ///< Voltage out of range
        ANGLE_LIMIT = 0x02,  ///< Angle exceeds limits
        OVERHEATING = 0x04,  ///< Temperature too high
        RANGE = 0x08,        ///< Command out of range
        CHECKSUM = 0x10,     ///< Packet checksum error
        OVERLOAD = 0x20,     ///< Servo overload
        INSTRUCTION = 0x40   ///< Invalid instruction
    };

    /**
     * @brief Status return level configuration
     */
    enum class status_level_t : uint8_t
    {
        NONE = 0,      ///< No return packet
        READ_ONLY = 1  ///< Return packet only for read commands
    };

    /**
     * @brief Contains current status information from a servo
     */
    struct servo_status_t
    {
        uint16_t position{0};          ///< Current position (0-1023)
        int16_t speed{0};              ///< Current speed (-32768 to 32767)
        int16_t load{0};               ///< Current load (-1000 to 1000)
        uint8_t voltage{0};            ///< Current voltage (0.1V units)
        uint8_t temperature{0};        ///< Current temperature (°C)
        bool isMoving{false};          ///< Movement status flag
        int16_t current{0};            ///< Current draw
        error_t error{error_t::NONE};  ///< Current error state
    };

    /**
     * @brief Custom error category for ST3215 servo errors
     */
    class servo_error_category : public std::error_category
    {
    public:
        /**
         * @brief Get the name of the error category
         * @return const char* The name of the category
         */
        const char* name() const noexcept override
        {
            return "st3215_servo";
        }

        /**
         * @brief Get the error message for a given error code
         * @param ev The error value
         * @return std::string The error message
         */
        std::string message(int ev) const override
        {
            switch (static_cast<error_t>(ev))
            {
            case error_t::NONE:
                return "No error";
            case error_t::VOLTAGE:
                return "Voltage out of range";
            case error_t::ANGLE_LIMIT:
                return "Angle exceeds limits";
            case error_t::OVERHEATING:
                return "Temperature too high";
            case error_t::RANGE:
                return "Command out of range";
            case error_t::CHECKSUM:
                return "Packet checksum error";
            case error_t::OVERLOAD:
                return "Servo overload";
            case error_t::INSTRUCTION:
                return "Invalid instruction";
            default:
                return "Unknown error";
            }
        }

        /**
         * @brief Get the singleton instance of the error category
         * @return const servo_error_category& The error category instance
         */
        static const servo_error_category& get()
        {
            static servo_error_category instance;
            return instance;
        }
    };

    /**
     * @brief Create an error code from a servo error
     * @param e The servo error
     * @return std::error_code The corresponding error code
     */
    inline std::error_code make_error_code(error_t e)
    {
        return {static_cast<int>(e), servo_error_category::get()};
    }

}  // namespace st3215

// Enable automatic conversion of error_t to error_code
namespace std
{
    template <>
    struct is_error_code_enum<st3215::error_t> : true_type
    {
    };
}
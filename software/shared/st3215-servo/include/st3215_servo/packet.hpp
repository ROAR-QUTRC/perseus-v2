#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "core.hpp"
#include "memory_tables.hpp"

namespace st3215
{
    /**
     * @brief Class for constructing and parsing ST3215 protocol packets
     */
    class Packet
    {
    public:
        /**
         * @brief Maximum packet size including header and checksum
         */
        static constexpr size_t MAX_PACKET_SIZE = 223;  // Based on protocol limits

        /**
         * @brief Minimum packet size (header + id + length + instruction + checksum)
         */
        static constexpr size_t MIN_PACKET_SIZE = 6;

        /**
         * @brief Constructor for creating a new packet
         * @param id Servo ID (0-253, 254 for broadcast)
         * @param instruction Instruction type
         * @param parameters Optional parameters for the instruction
         * @throws std::invalid_argument if packet construction would be invalid
         */
        Packet(uint8_t id, instruction_t instruction, std::span<const uint8_t> parameters = {});

        /**
         * @brief Create a packet for reading from servo memory
         * @param id Servo ID
         * @param address Memory address to read from
         * @param size Number of bytes to read
         * @return Packet The constructed read packet
         * @throws std::invalid_argument if parameters are invalid
         */
        static Packet createRead(uint8_t id, uint8_t address, uint8_t size);

        /**
         * @brief Create a packet for writing to servo memory
         * @param id Servo ID
         * @param address Memory address to write to
         * @param data Data to write
         * @return Packet The constructed write packet
         * @throws std::invalid_argument if parameters are invalid
         */
        static Packet createWrite(uint8_t id, uint8_t address, std::span<const uint8_t> data);

        /**
         * @brief Create a sync write packet for multiple servos
         * @param address Starting address to write to
         * @param servoData Vector of pairs containing servo ID and data to write
         * @return Packet The constructed sync write packet
         * @throws std::invalid_argument if parameters are invalid
         */
        static Packet createSyncWrite(uint8_t address,
                                      const std::vector<std::pair<uint8_t, std::vector<uint8_t>>>& servoData);

        /**
         * @brief Create a ping packet to check if a servo exists
         * @param id Servo ID to ping
         * @return Packet The constructed ping packet
         */
        static Packet createPing(uint8_t id);

        /**
         * @brief Parse raw packet data into a Packet object
         * @param data Raw packet data
         * @return Packet The parsed packet
         * @throws std::invalid_argument if packet data is invalid
         */
        static Packet parse(std::span<const uint8_t> data);

        /**
         * @brief Get the raw packet data
         * @return std::span<const uint8_t> View of the packet data
         */
        std::span<const uint8_t> getData() const { return _data; }

        /**
         * @brief Get the servo ID
         * @return uint8_t Servo ID
         */
        uint8_t getId() const { return _data[2]; }

        /**
         * @brief Get the instruction type
         * @return instruction_t Packet instruction
         */
        instruction_t getInstruction() const { return static_cast<instruction_t>(_data[4]); }

        /**
         * @brief Get the packet parameters
         * @return std::span<const uint8_t> View of the parameter data
         */
        std::span<const uint8_t> getParameters() const;

        /**
         * @brief Get any error code from a status packet
         * @return error_t Error code from the packet
         * @throws std::runtime_error if this is not a status packet
         */
        error_t getError() const;

        /**
         * @brief Check if this is a status packet (response from servo)
         * @return bool True if this is a status packet
         */
        bool isStatusPacket() const;

        /**
         * @brief Validate packet structure and checksum
         * @return bool True if packet is valid
         */
        bool isValid() const;

    private:
        /**
         * @brief Calculate packet checksum
         * @return uint8_t Calculated checksum
         */
        uint8_t _calculateChecksum() const;

        /**
         * @brief Validate servo ID
         * @param id ID to validate
         * @return bool True if ID is valid
         */
        static constexpr bool _isValidId(uint8_t id)
        {
            return id <= BROADCAST_ID;
        }

        /**
         * @brief Internal packet data storage
         */
        std::vector<uint8_t> _data;
    };

    /**
     * @brief Response packet from servo
     */
    class StatusPacket : public Packet
    {
    public:
        /**
         * @brief Parse a status packet from raw data
         * @param data Raw packet data
         * @return StatusPacket The parsed status packet
         * @throws std::invalid_argument if data is not a valid status packet
         */
        static StatusPacket parse(std::span<const uint8_t> data);

        /**
         * @brief Get the error flags from the status packet
         * @return error_t The error flags
         */
        error_t getError() const { return static_cast<error_t>(getParameters()[0]); }

        /**
         * @brief Get the parameter data (excluding error byte)
         * @return std::span<const uint8_t> View of the parameter data
         */
        std::span<const uint8_t> getParameterData() const;

    private:
        /**
         * @brief Construct from parent Packet
         */
        explicit StatusPacket(Packet&& packet) : Packet(std::move(packet)) {}
    };

}  // namespace st3215
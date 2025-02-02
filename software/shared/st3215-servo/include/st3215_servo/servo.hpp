#pragma once

#include <boost/asio.hpp>
#include <memory>
#include <optional>

#include "core.hpp"
#include "memory_tables.hpp"
#include "packet.hpp"

namespace st3215
{
    // Forward declaration of ServoManager for friend access
    class ServoManager;

    /**
     * @brief Class for controlling an individual ST3215 servo
     * @details Provides interface for controlling and monitoring a single servo.
     *          Thread-safe when accessed through ServoManager.
     */
    class Servo : public std::enable_shared_from_this<Servo>
    {
    public:
        /**
         * @brief Get the ID of this servo
         * @return uint8_t Servo ID
         */
        uint8_t getId() const { return _id; }

        /**
         * @brief Set target position
         * @param position Target position (0-1023)
         * @param speed Optional movement speed (0 for default)
         * @throws std::invalid_argument if position is out of range
         * @throws std::runtime_error if communication fails
         */
        void setPosition(uint16_t position, uint16_t speed = 0);

        /**
         * @brief Set wheel mode speed
         * @param speed Target speed (-1000 to 1000)
         * @throws std::invalid_argument if speed is out of range
         * @throws std::runtime_error if communication fails or wrong mode
         */
        void setSpeed(int16_t speed);

        /**
         * @brief Enable or disable torque
         * @param enable True to enable, false to disable
         * @throws std::runtime_error if communication fails
         */
        void setTorque(bool enable);

        /**
         * @brief Set operating mode
         * @param mode Desired operating mode
         * @throws std::runtime_error if communication fails
         */
        void setOperatingMode(operating_mode_t mode);

        /**
         * @brief Set position limits
         * @param min Minimum position (0-1023)
         * @param max Maximum position (0-1023)
         * @throws std::invalid_argument if limits are invalid
         * @throws std::runtime_error if communication fails
         */
        void setPositionLimits(uint16_t min, uint16_t max);

        /**
         * @brief Set maximum torque limit
         * @param limit Torque limit (0-1000)
         * @throws std::invalid_argument if limit is out of range
         * @throws std::runtime_error if communication fails
         */
        void setTorqueLimit(uint16_t limit);

        /**
         * @brief Get current servo status
         * @return servo_status_t Current status
         * @throws std::runtime_error if communication fails
         */
        servo_status_t getStatus();

        /**
         * @brief Check if servo is moving
         * @return bool True if servo responds
         * @throws std::runtime_error if communication fails
         */
        bool isMoving();

        /**
         * @brief Verify servo responds to communication
         * @param timeout_ms Timeout in milliseconds
         * @return bool True if servo responds
         */
        bool ping(uint32_t timeout_ms = 100);

        /**
         * @brief Read from servo memory
         * @param address Memory address to read
         * @param size Number of bytes to read (1 or 2)
         * @return std::vector<uint8_t> Read data
         * @throws std::invalid_argument if address or size is invalid
         * @throws std::runtime_error if communication fails
         */
        std::vector<uint8_t> read(uint8_t address, uint8_t size);

        /**
         * @brief Write to servo memory
         * @param address Memory address to write
         * @param data Data to write
         * @throws std::invalid_argument if address or data is invalid
         * @throws std::runtime_error if communication fails
         */
        void write(uint8_t address, std::span<const uint8_t> data);

        /**
         * @brief Get current operating mode
         * @return operating_mode_t Current mode
         * @throws std::runtime_error if communication fails
         */
        operating_mode_t getOperatingMode();

        /**
         * @brief Factory reset the servo
         * @param resetId True to reset ID to default (1)
         * @throws std::runtime_error if communication fails
         */
        void factoryReset(bool resetId = false);

    private:
        // Make constructor private but accessible to EnableMakeShared and ServoManager
        Servo(uint8_t id, ServoManager& manager);

        // Static helper struct for creation
        struct EnableMakeShared;

        static std::shared_ptr<Servo> create(uint8_t id, ServoManager& manager)
        {
            // This struct gives us access to the private constructor
            struct EnableMakeShared : public Servo
            {
                EnableMakeShared(uint8_t id, ServoManager& manager)
                    : Servo(id, manager) {}
            };
            return std::make_shared<EnableMakeShared>(id, manager);
        }

        /**
         * @brief Validate a servo ID
         * @param id ID to validate
         * @return true if ID is valid
         */
        static bool _isValidId(uint8_t id);

        /**
         * @brief Read a 2-byte value from memory
         * @param address Memory address to read
         * @return uint16_t Read value
         * @throws std::runtime_error if communication fails
         */
        uint16_t _read16(uint8_t address);

        /**
         * @brief Write a 2-byte value to memory
         * @param address Memory address to write
         * @param value Value to write
         * @throws std::runtime_error if communication fails
         */
        void _write16(uint8_t address, uint16_t value);

        /**
         * @brief Helper for range checking
         * @param value Value to check
         * @param min Minimum allowed value
         * @param max Maximum allowed value
         * @param name Parameter name for error message
         * @throws std::invalid_argument if value is out of range
         */
        template <typename T1, typename T2, typename T3>
        static void _checkRange(T1 value, T2 min, T3 max, const char* name)
        {
            if (value < min || value > max)
            {
                throw std::invalid_argument(
                    std::format("{} must be between {} and {}", name, min, max));
            }
        }

        uint8_t _id;                    ///< Servo ID
        ServoManager& _manager;         ///< Parent manager
        operating_mode_t _currentMode;  ///< Current operating mode cache

        friend class ServoManager;
    };

}  // namespace st3215
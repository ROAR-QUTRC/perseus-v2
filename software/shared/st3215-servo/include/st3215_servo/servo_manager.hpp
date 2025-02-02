#pragma once

#include <boost/asio.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "core.hpp"
#include "servo.hpp"

namespace st3215
{

    // Default timeout for servo communications (in milliseconds)
    static constexpr uint32_t DEFAULT_TIMEOUT = 100;

    /**
     * @brief Manager class for controlling multiple ST3215 servos
     * @details Provides thread-safe access to multiple servos on a single serial bus
     */
    class ServoManager
    {
    public:
        /**
         * @brief Constructor for ServoManager
         * @param port Serial port device path
         * @param baudRate Communication baud rate
         * @throws std::runtime_error if port cannot be opened
         */
        explicit ServoManager(const std::string& port = "/dev/ttyACM0",
                              uint32_t baudRate = 1000000);

        // Prevent copying - we manage unique serial port resources
        ServoManager(const ServoManager&) = delete;
        ServoManager& operator=(const ServoManager&) = delete;

        // Allow moving
        ServoManager(ServoManager&&) noexcept = default;
        ServoManager& operator=(ServoManager&&) noexcept = default;

        void sendPacket(const Packet& packet);
        std::optional<Packet> receivePacket(uint32_t timeout_ms = DEFAULT_TIMEOUT);

        /**
         * @brief Add a servo to the manager
         * @param id Servo ID (0-253)
         * @return std::shared_ptr<Servo> Pointer to the added servo
         * @throws std::invalid_argument if ID is invalid or already exists
         */
        std::shared_ptr<Servo> addServo(uint8_t id);

        /**
         * @brief Remove a servo from the manager
         * @param id Servo ID to remove
         * @throws std::invalid_argument if servo doesn't exist
         */
        void removeServo(uint8_t id);

        /**
         * @brief Get a servo by ID
         * @param id Servo ID to retrieve
         * @return std::shared_ptr<Servo> Pointer to the servo
         * @throws std::out_of_range if servo doesn't exist
         */
        std::shared_ptr<Servo> getServo(uint8_t id) const;

        /**
         * @brief Check if a servo ID exists in the manager
         * @param id Servo ID to check
         * @return bool True if servo exists
         */
        bool hasServo(uint8_t id) const;

        /**
         * @brief Get all managed servo IDs
         * @return std::vector<uint8_t> List of servo IDs
         */
        std::vector<uint8_t> getServoIds() const;

        /**
         * @brief Scan the bus for connected servos
         * @param startId Start of ID range to scan (inclusive)
         * @param endId End of ID range to scan (inclusive)
         * @param timeout_ms Timeout for each servo in milliseconds
         * @return std::vector<uint8_t> List of found servo IDs
         */
        std::vector<uint8_t> scanForServos(uint8_t startId = 0,
                                           uint8_t endId = MAX_SERVOS,
                                           uint32_t timeout_ms = 100);

        /**
         * @brief Synchronized position write to multiple servos
         * @param positions Vector of ID/position pairs
         * @param speed Optional movement speed (0 for default)
         * @throws std::invalid_argument if any servo ID is invalid
         */
        void syncWritePosition(const std::vector<std::pair<uint8_t, uint16_t>>& positions,
                               uint16_t speed = 0);

        /**
         * @brief Synchronized speed write to multiple servos
         * @param speeds Vector of ID/speed pairs
         * @throws std::invalid_argument if any servo ID is invalid
         */
        void syncWriteSpeed(const std::vector<std::pair<uint8_t, int16_t>>& speeds);

        /**
         * @brief Synchronized torque enable/disable for multiple servos
         * @param servos Vector of servo IDs to affect
         * @param enable True to enable torque, false to disable
         * @throws std::invalid_argument if any servo ID is invalid
         */
        void syncWriteTorque(const std::vector<uint8_t>& servos, bool enable);

        /**
         * @brief Emergency stop - disables torque on all servos
         */
        void emergencyStop();

        /**
         * @brief Get the underlying serial port
         * @return boost::asio::serial_port& Reference to the serial port
         */
        boost::asio::serial_port& getSerialPort() { return _port; }

    private:
        /**
         * @brief Initialize the serial port
         * @param port Port device path
         * @param baudRate Communication baud rate
         */
        void _initializePort(const std::string& port, uint32_t baudRate);

        /**
         * @brief Send a packet with thread safety
         * @param packet Packet to send
         */
        void _sendPacket(const Packet& packet);

        /**
         * @brief Receive a packet with thread safety
         * @param timeout_ms Timeout in milliseconds
         * @return std::optional<StatusPacket> Received packet, if any
         */
        std::optional<StatusPacket> _receivePacket(uint32_t timeout_ms = 100);

        boost::asio::io_context _io;
        boost::asio::serial_port _port;

        /// @brief Thread safety for serial port access
        mutable std::mutex _portMutex;

        /// @brief Map of servo ID to Servo object
        std::unordered_map<uint8_t, std::shared_ptr<Servo>> _servos;

        /// @brief Thread safety for servo map access
        mutable std::mutex _servosMutex;
    };

}  // namespace st3215
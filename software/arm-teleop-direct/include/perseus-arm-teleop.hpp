#pragma once

// Forward declare WINDOW type to avoid ncurses/boost conflicts
struct _win_st;
typedef struct _win_st WINDOW;

#include <fcntl.h>
#include <termios.h>

#include <boost/asio.hpp>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

class ST3215ServoReader
{
public:
    /**
     * @brief Constructs a new ST3215ServoReader
     * @param port Serial port path (e.g., "/dev/ttyACM0")
     * @param baud_rate Baud rate for serial communication
     * @param acceleration Acceleration value for servo movement (0-255)
     */
    ST3215ServoReader(const std::string& port, unsigned int baud_rate, uint8_t acceleration = 30);

    /**
     * @brief Destructor ensures serial port is properly closed
     */
    ~ST3215ServoReader();

    /**
     * @brief Reads the current position of a specified servo
     * @param servo_id ID of the servo to read from
     * @return Current position value (0-4095)
     * @throws std::runtime_error if communication fails after retries
     */
    uint16_t readPosition(uint8_t servo_id);

    /**
     * @brief Writes a position value to a specified servo
     * @param servo_id ID of the servo to write to
     * @param position Position value to write (0-4095)
     * @throws std::runtime_error if communication fails
     */
    void writePosition(uint8_t servo_id, uint16_t position);

    /**
     * @brief Writes to a control register of the servo
     * @param servo_id ID of the servo to write to
     * @param address Register address
     * @param value Value to write
     * @throws std::runtime_error if communication fails
     */
    void writeControlRegister(uint8_t servo_id, uint8_t address, uint8_t value);

    /**
     * @brief Reads the current load/torque value from a specified servo
     * @param servo_id ID of the servo to read from
     * @return Current load value (-1000 to +1000)
     * @throws std::runtime_error if communication fails
     */
    int16_t readLoad(uint8_t servo_id);

    /**
     * @brief Gets a reference to the serial port for low-level operations
     * @return Reference to the serial port object
     */
    boost::asio::serial_port& getSerialPort() { return _serial_port; }

private:
    /**
     * @brief Creates a write command packet according to ST3215 protocol
     * @param id Servo ID
     * @param address Memory address to write to
     * @param data Data bytes to write
     * @return Vector containing the complete command packet
     */
    std::vector<uint8_t> _createWriteCommand(uint8_t id, uint8_t address, const std::vector<uint8_t>& data);

    /**
     * @brief Performs a single attempt to read the position
     * @param servo_id ID of the servo to read from
     * @param timeout Maximum time to wait for response
     * @return Current position value (0-4095)
     * @throws std::runtime_error if communication fails
     */
    uint16_t _readPositionOnce(uint8_t servo_id, const std::chrono::milliseconds& timeout);

    /**
     * @brief Creates a read command packet according to ST3215 protocol
     * @param id Servo ID
     * @param address Memory address to read from
     * @param size Number of bytes to read
     * @return Vector containing the complete command packet
     */
    std::vector<uint8_t> _createReadCommand(uint8_t id, uint8_t address, uint8_t size);

    boost::asio::io_service _io_service;
    boost::asio::serial_port _serial_port;
};
#include "perseus-arm-teleop.hpp"

#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "servo-constants.hpp"

using namespace boost::asio;
using namespace perseus::servo;

ST3215ServoReader::ST3215ServoReader(const std::string& port, unsigned int baud_rate, uint8_t acceleration)
    : _io_service(),
      _serial_port(_io_service)
{
    try
    {
        _serial_port.open(port);

        // Get the native handle for low-level configuration
        int fd = _serial_port.native_handle();

        // Configure ACM port settings
        struct termios to;
        if (tcgetattr(fd, &to) != 0)
        {
            throw std::runtime_error("Failed to get port attributes");
        }

        // Input flags - disable break processing
        to.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

        // Output flags - disable post processing
        to.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

        // No line processing
        to.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

        // Clean character processing
        to.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB);
        to.c_cflag |= CS8 | CLOCAL | CREAD;

        // Set speed
        if (cfsetspeed(&to, baud_rate) != 0)
        {
            throw std::runtime_error("Failed to set baud rate");
        }

        // One byte at a time, no timer
        to.c_cc[VMIN] = 1;
        to.c_cc[VTIME] = 0;

        // Apply settings
        if (tcsetattr(fd, TCSANOW, &to) != 0)
        {
            throw std::runtime_error("Failed to apply port attributes");
        }

        // Clear all IO buffers
        if (tcflush(fd, TCIOFLUSH) != 0)
        {
            throw std::runtime_error("Failed to flush port");
        }

        // We still set the boost::asio options for consistency
        _serial_port.set_option(serial_port::baud_rate(baud_rate));
        _serial_port.set_option(serial_port::character_size(8));
        _serial_port.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        _serial_port.set_option(serial_port::parity(serial_port::parity::none));
        _serial_port.set_option(serial_port::flow_control(serial_port::flow_control::none));

        // Initial delay to let port settle - ACM devices often need more time
        std::this_thread::sleep_for(timing::PORT_SETTLE_TIME);

        // Set acceleration for all servos (1-6)
        for (uint8_t id = 1; id <= 6; ++id)
        {
            try
            {
                writeControlRegister(id, register_addr::ACCELERATION, acceleration);
                std::this_thread::sleep_for(timing::MIN_RESPONSE_TIME);  // Wait between writes
            }
            catch (const std::exception& e)
            {
                std::cerr << "Warning: Failed to set acceleration for servo " << static_cast<int>(id)
                          << ": " << e.what() << std::endl;
            }
        }
    }
    catch (const boost::system::system_error& e)
    {
        throw std::runtime_error(std::string("Failed to open serial port: ") + e.what());
    }
}

ST3215ServoReader::~ST3215ServoReader()
{
    try
    {
        if (_serial_port.is_open())
        {
            ::tcflush(static_cast<int>(_serial_port.native_handle()), TCIOFLUSH);
            _serial_port.close();
        }
    }
    catch (...)
    {
        // Ignore errors in destructor
    }
}

uint16_t ST3215ServoReader::readPosition(uint8_t servo_id)
{
    for (int retry = 0; retry < communication::MAX_RETRIES; ++retry)
    {
        try
        {
            return _readPositionOnce(servo_id, timing::DEFAULT_TIMEOUT);
        }
        catch (const std::runtime_error& e)
        {
            if (retry == communication::MAX_RETRIES - 1)
            {
                throw;  // Re-throw if this was our last retry
            }
            // Flush the port and wait before retry
            ::tcflush(static_cast<int>(_serial_port.native_handle()), TCIOFLUSH);
            std::this_thread::sleep_for(timing::RETRY_DELAY);
        }
    }
    throw std::runtime_error("Maximum retries exceeded");
}

#include <fcntl.h>
#include <termios.h>

uint16_t ST3215ServoReader::_readPositionOnce(uint8_t servo_id, const std::chrono::milliseconds& timeout)
{
    // Create read position command packet
    std::vector<uint8_t> command = _createReadCommand(servo_id, register_addr::PRESENT_POSITION, 2);

    // Clear any existing data and wait for port to clear
    ::tcflush(static_cast<int>(_serial_port.native_handle()), TCIOFLUSH);
    std::this_thread::sleep_for(timing::COMMAND_INTERVAL);

    // Send command with retry
    boost::system::error_code write_ec;
    size_t written = 0;
    int write_attempts = 0;
    const int MAX_WRITE_ATTEMPTS = 3;

    while (written != command.size() && write_attempts < MAX_WRITE_ATTEMPTS)
    {
        written = boost::asio::write(_serial_port, buffer(command), write_ec);
        if (write_ec || written != command.size())
        {
            write_attempts++;
            if (write_attempts < MAX_WRITE_ATTEMPTS)
            {
                std::this_thread::sleep_for(timing::MIN_RESPONSE_TIME);
                continue;
            }
        }
        break;
    }
    if (write_ec)
    {
        throw std::runtime_error(std::string("Write error: ") + write_ec.message());
    }
    if (written != command.size())
    {
        throw std::runtime_error("Failed to write complete command");
    }

    // Ensure minimum response time
    std::this_thread::sleep_for(timing::MIN_RESPONSE_TIME);

    // Read response using a fixed buffer
    std::array<uint8_t, 256> response_buffer;
    size_t total_read = 0;
    const size_t HEADER_SIZE = 4;
    auto start_time = std::chrono::steady_clock::now();

    // Read header with timeout
    while (total_read < HEADER_SIZE)
    {
        if (std::chrono::steady_clock::now() - start_time > timeout)
        {
            throw std::runtime_error("Timeout waiting for header");
        }

        boost::system::error_code read_ec;
        size_t bytes = _serial_port.read_some(
            buffer(response_buffer.data() + total_read, HEADER_SIZE - total_read),
            read_ec);

        if (read_ec)
        {
            if (read_ec == boost::asio::error::operation_aborted ||
                read_ec == boost::asio::error::interrupted)
            {
                continue;  // Retry on interruption
            }
            throw std::runtime_error(std::string("Header read error: ") + read_ec.message());
        }

        if (bytes > 0)
        {
            total_read += bytes;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    // Validate header
    if (response_buffer[0] != protocol::HEADER1 || response_buffer[1] != protocol::HEADER2)
    {
        throw std::runtime_error("Invalid header markers");
    }
    if (response_buffer[2] != servo_id)
    {
        throw std::runtime_error("Mismatched servo ID");
    }
    if (response_buffer[3] < 4)
    {
        throw std::runtime_error("Invalid length");
    }

    // Read remaining data
    const size_t remaining_bytes = response_buffer[3];
    total_read = 0;
    start_time = std::chrono::steady_clock::now();

    while (total_read < remaining_bytes)
    {
        if (std::chrono::steady_clock::now() - start_time > timeout)
        {
            throw std::runtime_error("Timeout waiting for data");
        }

        boost::system::error_code read_ec;
        size_t bytes = _serial_port.read_some(
            buffer(response_buffer.data() + HEADER_SIZE + total_read,
                   remaining_bytes - total_read),
            read_ec);

        if (read_ec)
        {
            if (read_ec == boost::asio::error::operation_aborted ||
                read_ec == boost::asio::error::interrupted)
            {
                continue;  // Retry on interruption
            }
            throw std::runtime_error(std::string("Data read error: ") + read_ec.message());
        }

        if (bytes > 0)
        {
            total_read += bytes;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    // Check for servo errors
    if (response_buffer[HEADER_SIZE] != 0x00)
    {
        std::string error = "Servo errors:";
        if (response_buffer[HEADER_SIZE] & error_bits::INPUT_VOLTAGE)
            error += " Input Voltage";
        if (response_buffer[HEADER_SIZE] & error_bits::ANGLE_LIMIT)
            error += " Angle Limit";
        if (response_buffer[HEADER_SIZE] & error_bits::OVERHEATING)
            error += " Overheating";
        if (response_buffer[HEADER_SIZE] & error_bits::RANGE)
            error += " Range";
        if (response_buffer[HEADER_SIZE] & error_bits::CHECKSUM)
            error += " Checksum";
        if (response_buffer[HEADER_SIZE] & error_bits::OVERLOAD)
            error += " Overload";
        if (response_buffer[HEADER_SIZE] & error_bits::INSTRUCTION)
            error += " Instruction";
        throw std::runtime_error(error);
    }

    // Position is in little-endian format
    return static_cast<uint16_t>(response_buffer[HEADER_SIZE + 1]) |
           (static_cast<uint16_t>(response_buffer[HEADER_SIZE + 2]) << 8);
}

std::vector<uint8_t> ST3215ServoReader::_createReadCommand(uint8_t id, uint8_t address, uint8_t size)
{
    std::vector<uint8_t> command = {
        protocol::HEADER1, protocol::HEADER2,  // Header
        id,                                    // Servo ID
        0x04,                                  // Length (always 4 for read command)
        instruction::READ,                     // READ instruction
        address,                               // Starting address
        size,                                  // Number of bytes to read
        0x00                                   // Checksum (to be calculated)
    };

    // Calculate checksum
    uint8_t checksum = 0;
    for (size_t i = 2; i < command.size() - 1; i++)
    {
        checksum += command[i];
    }
    command[command.size() - 1] = ~checksum;

    return command;
}

std::vector<uint8_t> ST3215ServoReader::_createWriteCommand(uint8_t id, uint8_t address, const std::vector<uint8_t>& data)
{
    std::vector<uint8_t> command;
    command.reserve(data.size() + 6);  // Pre-allocate space

    // Header
    command.push_back(protocol::HEADER1);
    command.push_back(protocol::HEADER2);
    command.push_back(id);
    command.push_back(data.size() + 3);     // Length (data size + 3 bytes for instruction, address, and checksum)
    command.push_back(instruction::WRITE);  // WRITE instruction
    command.push_back(address);

    // Data
    command.insert(command.end(), data.begin(), data.end());

    // Calculate checksum
    uint8_t checksum = 0;
    for (size_t i = 2; i < command.size(); i++)
    {
        checksum += command[i];
    }
    command.push_back(~checksum);

    return command;
}

void ST3215ServoReader::writePosition(uint8_t servo_id, uint16_t position)
{
    // Clamp position to valid range
    position = std::min(position, limits::MAX_POSITION);

    // Create data bytes (little-endian)
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(position & 0xFF),
        static_cast<uint8_t>((position >> 8) & 0xFF)};

    // Create write command packet
    std::vector<uint8_t> command = _createWriteCommand(servo_id, register_addr::GOAL_POSITION, data);

    // Send command
    boost::system::error_code write_ec;
    size_t written = boost::asio::write(_serial_port, buffer(command), write_ec);

    if (write_ec)
    {
        throw std::runtime_error(std::string("Write error: ") + write_ec.message());
    }

    if (written != command.size())
    {
        throw std::runtime_error("Failed to write complete command");
    }

    // Wait for minimum response time
    std::this_thread::sleep_for(timing::MIN_RESPONSE_TIME);

    // Read status packet
    std::array<uint8_t, 6> response;  // Status packet is 6 bytes
    size_t read = 0;

    try
    {
        read = boost::asio::read(_serial_port, buffer(response), write_ec);
    }
    catch (const boost::system::system_error& e)
    {
        // Ignore read errors as the servo might not respond
        return;
    }

    if (read >= 4 && response[0] == protocol::HEADER1 && response[1] == protocol::HEADER2 &&
        response[2] == servo_id && response[3] >= 2)
    {
        // Check for servo errors if we got a valid response
        if (response[4] != 0x00)
        {
            std::string error = "Servo errors:";
            if (response[4] & error_bits::INPUT_VOLTAGE)
                error += " Input Voltage";
            if (response[4] & error_bits::ANGLE_LIMIT)
                error += " Angle Limit";
            if (response[4] & error_bits::OVERHEATING)
                error += " Overheating";
            if (response[4] & error_bits::RANGE)
                error += " Range";
            if (response[4] & error_bits::CHECKSUM)
                error += " Checksum";
            if (response[4] & error_bits::OVERLOAD)
                error += " Overload";
            if (response[4] & error_bits::INSTRUCTION)
                error += " Instruction";
            throw std::runtime_error(error);
        }
    }
}

void ST3215ServoReader::writeControlRegister(uint8_t servo_id, uint8_t address, uint8_t value)
{
    std::vector<uint8_t> data = {value};
    std::vector<uint8_t> command = _createWriteCommand(servo_id, address, data);

    boost::system::error_code write_ec;
    size_t written = boost::asio::write(_serial_port, buffer(command), write_ec);

    if (write_ec)
    {
        throw std::runtime_error(std::string("Write error: ") + write_ec.message());
    }

    if (written != command.size())
    {
        throw std::runtime_error("Failed to write complete command");
    }

    // Wait for minimum response time
    std::this_thread::sleep_for(timing::MIN_RESPONSE_TIME);

    // Read status packet
    std::array<uint8_t, 6> response;
    try
    {
        boost::asio::read(_serial_port, buffer(response), write_ec);
    }
    catch (const boost::system::system_error& e)
    {
        // Ignore read errors as the servo might not respond
        return;
    }
}

int16_t ST3215ServoReader::readLoad(uint8_t servo_id)
{
    for (int retry = 0; retry < communication::MAX_RETRIES; ++retry)
    {
        try
        {
            // Create read load command packet
            std::vector<uint8_t> command = _createReadCommand(servo_id, register_addr::PRESENT_LOAD, 2);

            // Clear any existing data
            ::tcflush(static_cast<int>(_serial_port.native_handle()), TCIOFLUSH);
            std::this_thread::sleep_for(timing::COMMAND_INTERVAL);

            // Send command
            boost::system::error_code write_ec;
            size_t written = boost::asio::write(_serial_port, buffer(command), write_ec);

            if (write_ec || written != command.size())
            {
                throw std::runtime_error("Failed to write command");
            }

            // Ensure minimum response time
            std::this_thread::sleep_for(timing::MIN_RESPONSE_TIME);

            // Read response
            std::array<uint8_t, 8> response;  // Header(4) + Error(1) + Data(2) + Checksum(1)
            size_t read = boost::asio::read(_serial_port, buffer(response), write_ec);

            if (write_ec || read != response.size())
            {
                throw std::runtime_error("Failed to read response");
            }

            // Validate response
            if (response[0] != protocol::HEADER1 || response[1] != protocol::HEADER2 || response[2] != servo_id)
            {
                throw std::runtime_error("Invalid response header");
            }

            // Check for servo errors
            if (response[4] != 0x00)
            {
                std::stringstream ss;
                ss << "Servo reported error 0x" << std::hex << static_cast<int>(response[4]);
                throw std::runtime_error(ss.str());
            }

            // Load value is in little-endian format
            return static_cast<int16_t>(response[5] | (response[6] << 8));
        }
        catch (const std::runtime_error& e)
        {
            if (retry == communication::MAX_RETRIES - 1)
            {
                throw;
            }
            ::tcflush(static_cast<int>(_serial_port.native_handle()), TCIOFLUSH);
            std::this_thread::sleep_for(timing::RETRY_DELAY);
        }
    }
    throw std::runtime_error("Maximum retries exceeded");
}
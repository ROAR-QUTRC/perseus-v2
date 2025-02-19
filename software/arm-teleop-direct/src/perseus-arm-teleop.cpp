#include "perseus-arm-teleop.hpp"

#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

using namespace boost::asio;

ST3215ServoReader::ST3215ServoReader(const std::string& port, unsigned int baud_rate, uint8_t acceleration)
    : _io_service(), _serial_port(_io_service)
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
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Set acceleration for all servos (1-6)
        for (uint8_t id = 1; id <= 6; ++id)
        {
            try
            {
                writeControlRegister(id, 0x29, acceleration);                // 0x29 is acceleration register
                std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Wait between writes
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
    const int MAX_RETRIES = 3;
    const auto timeout = std::chrono::milliseconds(200);  // Increased timeout for ACM
    for (int retry = 0; retry < MAX_RETRIES; ++retry)
    {
        try
        {
            return _readPositionOnce(servo_id, timeout);
        }
        catch (const std::runtime_error& e)
        {
            if (retry == MAX_RETRIES - 1)
            {
                throw;  // Re-throw if this was our last retry
            }
            // Flush the port and wait before retry
            ::tcflush(static_cast<int>(_serial_port.native_handle()), TCIOFLUSH);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Longer delay between retries
        }
    }
    throw std::runtime_error("Maximum retries exceeded");
}

#include <fcntl.h>
#include <termios.h>

uint16_t ST3215ServoReader::_readPositionOnce(uint8_t servo_id, const std::chrono::milliseconds& timeout)
{
    // Create read position command packet
    std::vector<uint8_t> command = _createReadCommand(servo_id, 0x38, 2);

    // Clear any existing data and wait for port to clear
    ::tcflush(static_cast<int>(_serial_port.native_handle()), TCIOFLUSH);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

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
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
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

    // Ensure minimum response time - ST3215 needs at least 10ms
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

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
    if (response_buffer[0] != 0xFF || response_buffer[1] != 0xFF)
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
        if (response_buffer[HEADER_SIZE] & 0x01)
            error += " Input Voltage";
        if (response_buffer[HEADER_SIZE] & 0x02)
            error += " Angle Limit";
        if (response_buffer[HEADER_SIZE] & 0x04)
            error += " Overheating";
        if (response_buffer[HEADER_SIZE] & 0x08)
            error += " Range";
        if (response_buffer[HEADER_SIZE] & 0x10)
            error += " Checksum";
        if (response_buffer[HEADER_SIZE] & 0x20)
            error += " Overload";
        if (response_buffer[HEADER_SIZE] & 0x40)
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
        0xFF, 0xFF,  // Header
        id,          // Servo ID
        0x04,        // Length (always 4 for read command)
        0x02,        // READ instruction
        address,     // Starting address
        size,        // Number of bytes to read
        0x00         // Checksum (to be calculated)
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
    command.push_back(0xFF);
    command.push_back(0xFF);
    command.push_back(id);
    command.push_back(data.size() + 3);  // Length (data size + 3 bytes for instruction, address, and checksum)
    command.push_back(0x03);             // WRITE instruction
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
    position = std::min(position, static_cast<uint16_t>(4095));

    // Create data bytes (little-endian)
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(position & 0xFF),
        static_cast<uint8_t>((position >> 8) & 0xFF)};

    // Create write command packet
    std::vector<uint8_t> command = _createWriteCommand(servo_id, 0x2A, data);  // 0x2A is goal position address

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
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

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

    if (read >= 4 && response[0] == 0xFF && response[1] == 0xFF &&
        response[2] == servo_id && response[3] >= 2)
    {
        // Check for servo errors if we got a valid response
        if (response[4] != 0x00)
        {
            std::string error = "Servo errors:";
            if (response[4] & 0x01)
                error += " Input Voltage";
            if (response[4] & 0x02)
                error += " Angle Limit";
            if (response[4] & 0x04)
                error += " Overheating";
            if (response[4] & 0x08)
                error += " Range";
            if (response[4] & 0x10)
                error += " Checksum";
            if (response[4] & 0x20)
                error += " Overload";
            if (response[4] & 0x40)
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
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

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

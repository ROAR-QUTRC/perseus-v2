#include <boost/asio.hpp>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "perseus-arm-teleop-mini.hpp"

ST3215ServoReaderMini::ST3215ServoReaderMini(const std::string& port, unsigned int baud_rate)
    : _io_service(),
      _serial_port(_io_service, port)
{
    try
    {
        // Set port options
        _serial_port.set_option(boost::asio::serial_port::baud_rate(baud_rate));
        _serial_port.set_option(boost::asio::serial_port::character_size(8));
        _serial_port.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        _serial_port.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        _serial_port.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));

        // Allow time for the serial connection to stabilize
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    catch (const boost::system::system_error& e)
    {
        throw std::runtime_error(std::string("Failed to open serial port: ") + e.what());
    }
}

ST3215ServoReaderMini::~ST3215ServoReaderMini()
{
    if (_serial_port.is_open())
    {
        _serial_port.close();
    }
}

uint16_t ST3215ServoReaderMini::readPosition(uint8_t servo_id)
{
    const int MAX_RETRIES = 3;

    for (int retry = 0; retry < MAX_RETRIES; ++retry)
    {
        try
        {
            // Create read position command (address 0x38 = position register, 2 bytes)
            std::vector<uint8_t> command = _createReadCommand(servo_id, 0x38, 2);

            // Clear any existing data
            boost::asio::write(_serial_port, boost::asio::buffer(command));

            // Ensure minimum response time
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            // Read response - 8 bytes: header(2) + id(1) + length(1) + error(1) + data(2) + checksum(1)
            std::vector<uint8_t> response(8);
            size_t bytes_read = boost::asio::read(_serial_port, boost::asio::buffer(response));

            if (bytes_read != 8)
            {
                throw std::runtime_error("Incomplete response received");
            }

            // Validate response
            if (response[0] != 0xFF || response[1] != 0xFF)
            {
                throw std::runtime_error("Invalid header markers");
            }

            if (response[2] != servo_id)
            {
                throw std::runtime_error("Response ID mismatch");
            }

            // Check for errors
            if (response[4] != 0)
            {
                std::string error = "Servo error code: " + std::to_string(response[4]);
                throw std::runtime_error(error);
            }

            // Position is in the response (little-endian format)
            uint16_t position = response[5] | (response[6] << 8);
            return position;
        }
        catch (const std::exception& e)
        {
            if (retry == MAX_RETRIES - 1)
            {
                throw;  // Re-throw on last retry
            }

            // Wait before retry
            std::this_thread::sleep_for(std::chrono::milliseconds(50 * (retry + 1)));
        }
    }

    // This should never be reached due to the throw above
    throw std::runtime_error("Failed to read position after maximum retries");
}

void ST3215ServoReaderMini::writePosition(uint8_t servo_id, uint16_t position)
{
    // Clamp position to valid range (0-4095)
    position = std::min(position, static_cast<uint16_t>(4095));

    // Create data for the command (little-endian)
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(position & 0xFF),
        static_cast<uint8_t>((position >> 8) & 0xFF)};

    // Create write command (address 0x2A = goal position register)
    std::vector<uint8_t> command = _createWriteCommand(servo_id, 0x2A, data);

    // Send command
    boost::asio::write(_serial_port, boost::asio::buffer(command));

    // Wait for minimum response time
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Optionally read status packet, but we'll ignore it for simplicity
    if (_serial_port.available())
    {
        std::vector<uint8_t> response(6);  // Status packet is typically 6 bytes
        boost::asio::read(_serial_port, boost::asio::buffer(response));
    }
}

std::vector<uint8_t> ST3215ServoReaderMini::_createReadCommand(uint8_t id, uint8_t address, uint8_t size)
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

    // Calculate checksum (inverse of sum of all bytes excluding header and checksum)
    uint8_t checksum = 0;
    for (size_t i = 2; i < command.size() - 1; i++)
    {
        checksum += command[i];
    }
    command[command.size() - 1] = ~checksum;

    return command;
}

std::vector<uint8_t> ST3215ServoReaderMini::_createWriteCommand(uint8_t id, uint8_t address, const std::vector<uint8_t>& data)
{
    // Length is data length + 3 (for instruction, address, and checksum)
    uint8_t length = static_cast<uint8_t>(data.size() + 3);

    std::vector<uint8_t> command = {
        0xFF, 0xFF,  // Header
        id,          // Servo ID
        length,      // Length
        0x03,        // WRITE instruction
        address      // Starting address
    };

    // Add data bytes
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
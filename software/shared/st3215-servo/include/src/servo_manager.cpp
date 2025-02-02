#include "st3215_servo/servo_manager.hpp"

#include <chrono>
#include <format>
#include <thread>

using namespace st3215;
using namespace boost::asio;
using namespace std::chrono_literals;

ServoManager::ServoManager(const std::string& port, uint32_t baudRate)
    : _io(),
      _port(_io)
{
    _initializePort(port, baudRate);
}

void ServoManager::_initializePort(const std::string& port, uint32_t baudRate)
{
    try
    {
        _port.open(port);

        // Configure port settings
        _port.set_option(serial_port_base::baud_rate(baudRate));
        _port.set_option(serial_port_base::character_size(8));
        _port.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        _port.set_option(serial_port_base::parity(serial_port_base::parity::none));
        _port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

// Set port timeouts
// Linux-specific, might need platform-specific handling
#if defined(__linux__)
        int fd = _port.native_handle();
        struct termios to;
        tcgetattr(fd, &to);
        to.c_cc[VTIME] = 1;  // 0.1 second timeout
        to.c_cc[VMIN] = 0;   // Return immediately if no data
        tcsetattr(fd, TCSANOW, &to);
#endif
    }
    catch (const boost::system::system_error& e)
    {
        throw std::runtime_error(
            std::format("Failed to open serial port {}: {}", port, e.what()));
    }
}

std::shared_ptr<Servo> ServoManager::addServo(uint8_t id)
{
    std::lock_guard<std::mutex> lock(_servosMutex);

    if (_servos.contains(id))
    {
        throw std::invalid_argument(
            std::format("Servo with ID {} already exists", id));
    }

    auto servo = std::make_shared<Servo>(id, *this);

    // Verify servo responds before adding
    if (!servo->ping())
    {
        throw std::runtime_error(
            std::format("No response from servo {}", id));
    }

    _servos[id] = servo;
    return servo;
}

void ServoManager::removeServo(uint8_t id)
{
    std::lock_guard<std::mutex> lock(_servosMutex);

    if (!_servos.erase(id))
    {
        throw std::invalid_argument(
            std::format("No servo with ID {} found", id));
    }
}

std::shared_ptr<Servo> ServoManager::getServo(uint8_t id) const
{
    std::lock_guard<std::mutex> lock(_servosMutex);

    auto it = _servos.find(id);
    if (it == _servos.end())
    {
        throw std::out_of_range(
            std::format("No servo with ID {} found", id));
    }

    return it->second;
}

bool ServoManager::hasServo(uint8_t id) const
{
    std::lock_guard<std::mutex> lock(_servosMutex);
    return _servos.contains(id);
}

std::vector<uint8_t> ServoManager::getServoIds() const
{
    std::lock_guard<std::mutex> lock(_servosMutex);

    std::vector<uint8_t> ids;
    ids.reserve(_servos.size());

    for (const auto& [id, _] : _servos)
    {
        ids.push_back(id);
    }

    return ids;
}

std::vector<uint8_t> ServoManager::scanForServos(uint8_t startId,
                                                 uint8_t endId,
                                                 uint32_t timeout_ms)
{
    if (startId > endId || endId > MAX_SERVOS)
    {
        throw std::invalid_argument(
            std::format("Invalid ID range: {}-{}", startId, endId));
    }

    std::vector<uint8_t> foundIds;

    for (uint8_t id = startId; id <= endId; ++id)
    {
        auto packet = Packet::createPing(id);
        try
        {
            _sendPacket(packet);
            auto response = _receivePacket(timeout_ms);
            if (response && response->getId() == id)
            {
                foundIds.push_back(id);
            }
        }
        catch (const std::exception&)
        {
            // Continue scanning on error
            continue;
        }
    }

    return foundIds;
}

void ServoManager::syncWritePosition(
    const std::vector<std::pair<uint8_t, uint16_t>>& positions,
    uint16_t speed)
{
    if (positions.empty())
    {
        return;
    }

    // Prepare data for each servo
    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> servoData;
    servoData.reserve(positions.size());

    for (const auto& [id, position] : positions)
    {
        if (position > memory_table::eprom_t::POSITION_MAX)
        {
            throw std::invalid_argument(
                std::format("Invalid position {} for servo {}", position, id));
        }

        // Position (2 bytes) + speed (2 bytes) if specified
        std::vector<uint8_t> data;
        data.reserve(speed > 0 ? 4 : 2);

        // Position (little-endian)
        data.push_back(position & 0xFF);
        data.push_back((position >> 8) & 0xFF);

        if (speed > 0)
        {
            data.push_back(speed & 0xFF);
            data.push_back((speed >> 8) & 0xFF);
        }

        servoData.emplace_back(id, std::move(data));
    }

    auto packet = Packet::createSyncWrite(
        memory_table::sram_t::GOAL_POSITION,
        servoData);

    _sendPacket(packet);
    // No response expected for sync write
}

void ServoManager::syncWriteSpeed(
    const std::vector<std::pair<uint8_t, int16_t>>& speeds)
{
    if (speeds.empty())
    {
        return;
    }

    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> servoData;
    servoData.reserve(speeds.size());

    for (const auto& [id, speed] : speeds)
    {
        if (speed < memory_table::sram_t::SPEED_MIN ||
            speed > memory_table::sram_t::SPEED_MAX)
        {
            throw std::invalid_argument(
                std::format("Invalid speed {} for servo {}", speed, id));
        }

        std::vector<uint8_t> data;
        data.reserve(2);

        // Speed (little-endian)
        data.push_back(speed & 0xFF);
        data.push_back((speed >> 8) & 0xFF);

        servoData.emplace_back(id, std::move(data));
    }

    auto packet = Packet::createSyncWrite(
        memory_table::sram_t::GOAL_SPEED,
        servoData);

    _sendPacket(packet);
}

void ServoManager::syncWriteTorque(
    const std::vector<uint8_t>& servos,
    bool enable)
{
    if (servos.empty())
    {
        return;
    }

    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> servoData;
    servoData.reserve(servos.size());

    for (auto id : servos)
    {
        servoData.emplace_back(id, std::vector<uint8_t>{static_cast<uint8_t>(enable)});
    }

    auto packet = Packet::createSyncWrite(
        memory_table::sram_t::TORQUE_ENABLE,
        servoData);

    _sendPacket(packet);
}

void ServoManager::emergencyStop()
{
    // Create broadcast packet to disable all torque
    auto packet = Packet::createWrite(
        BROADCAST_ID,
        memory_table::sram_t::TORQUE_ENABLE,
        std::vector<uint8_t>{0});

    _sendPacket(packet);
}

void ServoManager::_sendPacket(const Packet& packet)
{
    std::lock_guard<std::mutex> lock(_portMutex);

    try
    {
        write(_port, buffer(packet.getData()));
    }
    catch (const boost::system::system_error& e)
    {
        throw std::runtime_error(
            std::format("Failed to send packet: {}", e.what()));
    }
}

std::optional<StatusPacket> ServoManager::_receivePacket(uint32_t timeout_ms)
{
    std::lock_guard<std::mutex> lock(_portMutex);

    std::vector<uint8_t> buffer;
    buffer.reserve(Packet::MAX_PACKET_SIZE);

    auto startTime = std::chrono::steady_clock::now();

    // Read until we have a complete packet or timeout
    while (true)
    {
        if (std::chrono::steady_clock::now() - startTime >
            std::chrono::milliseconds(timeout_ms))
        {
            return std::nullopt;
        }

        uint8_t byte;
        boost::system::error_code error;

        size_t bytesRead = read(_port, boost::asio::buffer(&byte, 1), error);

        if (error)
        {
            if (error == boost::asio::error::would_block)
            {
                std::this_thread::sleep_for(1ms);
                continue;
            }
            throw std::runtime_error(
                std::format("Failed to receive packet: {}", error.message()));
        }

        if (bytesRead == 0)
        {
            std::this_thread::sleep_for(1ms);
            continue;
        }

        buffer.push_back(byte);

        // Process buffer once we have enough bytes for a potential packet
        if (buffer.size() >= 4)
        {
            // Check for valid header
            if (buffer[0] != PACKET_HEADER[0] || buffer[1] != PACKET_HEADER[1])
            {
                // Invalid header, remove first byte and continue
                buffer.erase(buffer.begin());
                continue;
            }

            // Check if we have a complete packet
            if (buffer.size() >= 4 && buffer.size() >= buffer[3] + 4)
            {
                try
                {
                    return StatusPacket::parse(buffer);
                }
                catch (const std::exception&)
                {
                    // Invalid packet, remove first byte and continue
                    buffer.erase(buffer.begin());
                }
            }
        }
    }
}
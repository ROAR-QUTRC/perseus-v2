#include "st3215_servo/servo.hpp"

#include <format>
#include <stdexcept>
#include <vector>

#include "st3215_servo/servo_manager.hpp"

using namespace st3215;
using namespace memory_table;

// Maximum number of servos supported by protocol
static constexpr uint8_t MAX_SERVOS = 253;

bool Servo::_isValidId(uint8_t id)
{
    return id <= MAX_SERVOS || id == BROADCAST_ID;
}

Servo::Servo(uint8_t id, ServoManager& manager)
    : _id(id),
      _manager(manager),
      _currentMode(operating_mode_t::POSITION)  // Will be updated on first use
{
    if (!_isValidId(id))
    {
        throw std::invalid_argument(
            std::format("Invalid servo ID: {}. Must be between 0 and {}", id, MAX_SERVOS));
    }
}

void Servo::setPosition(uint16_t position, uint16_t speed)
{
    // Verify we're in position mode
    auto mode = getOperatingMode();
    if (mode != operating_mode_t::POSITION)
    {
        throw std::runtime_error("Servo must be in position mode for position control");
    }

    _checkRange(position, eprom_t::POSITION_MIN, eprom_t::POSITION_MAX, "Position");

    // If speed is specified, set it first
    if (speed > 0)
    {
        _write16(sram_t::GOAL_SPEED, speed);
    }

    _write16(sram_t::GOAL_POSITION, position);
}

void Servo::setSpeed(int16_t speed)
{
    // Verify we're in speed mode
    auto mode = getOperatingMode();
    if (mode != operating_mode_t::SPEED)
    {
        throw std::runtime_error("Servo must be in speed mode for direct speed control");
    }

    _checkRange(speed, sram_t::SPEED_MIN, sram_t::SPEED_MAX, "Speed");
    _write16(sram_t::GOAL_SPEED, speed);
}

void Servo::setTorque(bool enable)
{
    std::vector<uint8_t> data{static_cast<uint8_t>(enable ? 1 : 0)};
    write(sram_t::TORQUE_ENABLE, data);
}

void Servo::setOperatingMode(operating_mode_t mode)
{
    // Must disable torque to change mode
    bool wasTorqueEnabled = read(sram_t::TORQUE_ENABLE, 1)[0] != 0;
    if (wasTorqueEnabled)
    {
        setTorque(false);
    }

    std::vector<uint8_t> data{static_cast<uint8_t>(mode)};
    write(eprom_t::OPERATING_MODE, data);
    _currentMode = mode;

    // Restore torque if it was enabled
    if (wasTorqueEnabled)
    {
        setTorque(true);
    }
}

void Servo::setPositionLimits(uint16_t min, uint16_t max)
{
    _checkRange(min, eprom_t::POSITION_MIN, eprom_t::POSITION_MAX, "Minimum position");
    _checkRange(max, eprom_t::POSITION_MIN, eprom_t::POSITION_MAX, "Maximum position");

    if (min >= max)
    {
        throw std::invalid_argument("Minimum position must be less than maximum position");
    }

    _write16(eprom_t::MIN_POSITION_LIMIT, min);
    _write16(eprom_t::MAX_POSITION_LIMIT, max);
}

void Servo::setTorqueLimit(uint16_t limit)
{
    _checkRange(limit, 0, eprom_t::TORQUE_MAX, "Torque limit");
    _write16(eprom_t::MAX_TORQUE_LIMIT, limit);
}

servo_status_t Servo::getStatus()
{
    servo_status_t status;

    // Read all status registers in sequence
    status.position = _read16(sram_status_t::PRESENT_POSITION);
    status.speed = _read16(sram_status_t::PRESENT_SPEED);
    status.load = _read16(sram_status_t::PRESENT_LOAD);
    status.voltage = read(sram_status_t::PRESENT_VOLTAGE, 1)[0];
    status.temperature = read(sram_status_t::PRESENT_TEMPERATURE, 1)[0];
    status.isMoving = read(sram_status_t::MOVING_STATUS, 1)[0] != 0;
    status.current = _read16(sram_status_t::PRESENT_CURRENT);

    return status;
}

bool Servo::isMoving()
{
    return read(sram_status_t::MOVING_STATUS, 1)[0] != 0;
}

bool Servo::ping(uint32_t timeout_ms)
{
    try
    {
        auto packet = Packet::createPing(_id);
        _manager.sendPacket(packet);

        auto response = _manager.receivePacket(timeout_ms);
        return response.has_value() && response->getId() == _id;
    }
    catch (const std::exception&)
    {
        return false;
    }
}

std::vector<uint8_t> Servo::read(uint8_t address, uint8_t size)
{
    auto packet = Packet::createRead(_id, address, size);
    _manager.sendPacket(packet);
    auto response = _manager.receivePacket();

    if (!response)
    {
        throw std::runtime_error(
            std::format("Failed to read from servo {} at address {:#x}", _id, address));
    }

    // Convert to StatusPacket to access parameter data
    StatusPacket statusPacket = StatusPacket::parse(response->getData());

    // Check for servo errors
    auto error = statusPacket.getError();
    if (error != error_t::NONE)
    {
        throw std::runtime_error(
            std::format("Servo {} reported error: {}", _id, static_cast<int>(error)));
    }

    return std::vector<uint8_t>(
        statusPacket.getParameterData().begin(),
        statusPacket.getParameterData().end());
}

void Servo::write(uint8_t address, std::span<const uint8_t> data)
{
    auto packet = Packet::createWrite(_id, address, data);

    if (_id != BROADCAST_ID)
    {
        // Wait for response unless it's a broadcast
        _manager.sendPacket(packet);
        auto response = _manager.receivePacket();

        if (!response)
        {
            throw std::runtime_error(
                std::format("Failed to write to servo {} at address {:#x}", _id, address));
        }

        // Check for servo errors
        auto error = response->getError();
        if (error != error_t::NONE)
        {
            throw std::runtime_error(
                std::format("Servo {} reported error: {}", _id, static_cast<int>(error)));
        }
    }
    else
    {
        // For broadcast, just send the packet
        _manager.sendPacket(packet);
    }
}

operating_mode_t Servo::getOperatingMode()
{
    auto data = read(eprom_t::OPERATING_MODE, 1);
    _currentMode = static_cast<operating_mode_t>(data[0]);
    return _currentMode;
}

void Servo::factoryReset(bool resetId)
{
    // First disable torque
    setTorque(false);

    // Reset all parameters except ID
    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> defaults = {
        {eprom_t::BAUD_RATE, {0x04}},  // 1000000 bps
        {eprom_t::RETURN_DELAY_TIME, {250}},
        {eprom_t::MIN_POSITION_LIMIT, {0x00, 0x00}},
        {eprom_t::MAX_POSITION_LIMIT, {0xFF, 0x03}},  // 1023
        {eprom_t::MAX_TORQUE_LIMIT, {0xE8, 0x03}},    // 1000
        {eprom_t::STATUS_RETURN_LEVEL, {0x01}},
        {eprom_t::OPERATING_MODE, {0x00}},  // Position mode
    };

    if (resetId)
    {
        defaults.push_back({eprom_t::ID, {0x01}});  // Reset to ID 1
    }

    // Apply each default setting
    for (const auto& [address, value] : defaults)
    {
        write(address, value);
    }

    _currentMode = operating_mode_t::POSITION;
}

uint16_t Servo::_read16(uint8_t address)
{
    auto data = read(address, 2);
    return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

void Servo::_write16(uint8_t address, uint16_t value)
{
    std::vector<uint8_t> data{
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF)};
    write(address, data);
}
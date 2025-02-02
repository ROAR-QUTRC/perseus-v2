#include "st3215_servo/packet.hpp"

#include <algorithm>
#include <format>
#include <numeric>
#include <stdexcept>

using namespace st3215;

Packet::Packet(uint8_t id, instruction_t instruction, std::span<const uint8_t> parameters)
{
    if (!_isValidId(id))
    {
        throw std::invalid_argument(
            std::format("Invalid servo ID: {}. Must be between 0 and {}",
                        id, BROADCAST_ID));
    }

    const size_t param_size = parameters.size();
    // Check if param_size + 2 can safely fit in uint8_t before conversion
    if (param_size > static_cast<size_t>(std::numeric_limits<uint8_t>::max() - 2))
    {
        throw std::invalid_argument("Parameters too large for packet");
    }

    const uint8_t length = static_cast<uint8_t>(param_size + 2);  // +2 for instruction and checksum
    const size_t total_size = static_cast<size_t>(length) + 4;    // +4 for header(2), id, and length

    if (total_size > static_cast<size_t>(MAX_PACKET_SIZE))
    {
        throw std::invalid_argument(
            std::format("Packet too large: {}. Maximum size is {}",
                        total_size, MAX_PACKET_SIZE));
    }

    // Construct packet: [0xFF, 0xFF, ID, Length, Instruction, Parameters, Checksum]
    _data.reserve(total_size);

    // Header
    _data.insert(_data.end(), PACKET_HEADER.begin(), PACKET_HEADER.end());

    // ID, Length, Instruction
    _data.push_back(id);
    _data.push_back(length);
    _data.push_back(static_cast<uint8_t>(instruction));

    // Parameters
    _data.insert(_data.end(), parameters.begin(), parameters.end());

    // Checksum
    _data.push_back(_calculateChecksum());
}
Packet Packet::createRead(uint8_t id, uint8_t address, uint8_t size)
{
    if (size == 0 || size > memory_table::get_register_size(address))
    {
        throw std::invalid_argument(
            std::format("Invalid read size: {} for address {:#x}", size, address));
    }

    std::vector<uint8_t> parameters{address, size};
    return Packet(id, instruction_t::READ, parameters);
}

Packet Packet::createWrite(uint8_t id, uint8_t address, std::span<const uint8_t> data)
{
    if (data.empty())
    {
        throw std::invalid_argument("Write data cannot be empty");
    }

    if (data.size() > memory_table::get_register_size(address))
    {
        throw std::invalid_argument(
            std::format("Write size {} exceeds register size for address {:#x}",
                        data.size(), address));
    }

    std::vector<uint8_t> parameters;
    parameters.reserve(data.size() + 1);
    parameters.push_back(address);
    parameters.insert(parameters.end(), data.begin(), data.end());

    return Packet(id, instruction_t::WRITE, parameters);
}

Packet Packet::createSyncWrite(uint8_t address,
                               const std::vector<std::pair<uint8_t, std::vector<uint8_t>>>& servoData)
{
    if (servoData.empty())
    {
        throw std::invalid_argument("Sync write data cannot be empty");
    }

    // Verify all data chunks are the same size
    const size_t dataSize = servoData[0].second.size();
    if (dataSize == 0 || dataSize > memory_table::get_register_size(address))
    {
        throw std::invalid_argument(
            std::format("Invalid data size {} for address {:#x}", dataSize, address));
    }

    std::vector<uint8_t> parameters;
    parameters.reserve(2 + servoData.size() * (dataSize + 1));  // address + size + (id + data) per servo

    // Add address and data length
    parameters.push_back(address);
    parameters.push_back(dataSize);

    // Add servo IDs and data
    for (const auto& [id, data] : servoData)
    {
        if (!_isValidId(id))
        {
            throw std::invalid_argument(
                std::format("Invalid servo ID: {}", id));
        }
        if (data.size() != dataSize)
        {
            throw std::invalid_argument(
                std::format("Inconsistent data size for servo {}: expected {}, got {}",
                            id, dataSize, data.size()));
        }

        parameters.push_back(id);
        parameters.insert(parameters.end(), data.begin(), data.end());
    }

    return Packet(BROADCAST_ID, instruction_t::SYNC_WRITE, parameters);
}

Packet Packet::createPing(uint8_t id)
{
    return Packet(id, instruction_t::PING);
}

Packet Packet::parse(std::span<const uint8_t> data)
{
    const size_t data_size = data.size();
    // Validate minimum size
    if (data_size < static_cast<size_t>(MIN_PACKET_SIZE))
    {
        throw std::invalid_argument(
            std::format("Packet too small: {}. Minimum size is {}",
                        data_size, MIN_PACKET_SIZE));
    }

    // Validate header
    if (data[0] != PACKET_HEADER[0] || data[1] != PACKET_HEADER[1])
    {
        throw std::invalid_argument("Invalid packet header");
    }

    // Validate length
    const uint8_t length = data[3];
    const size_t expected_size = static_cast<size_t>(length) + 4;  // Total size = length + header(2) + id + length

    if (length < 2)  // Must have at least instruction and checksum
    {
        throw std::invalid_argument(
            std::format("Invalid packet length: {}", length));
    }
    if (data_size != expected_size)
    {
        throw std::invalid_argument(
            std::format("Packet size mismatch: expected {}, got {}",
                        expected_size, data_size));
    }

    // Create packet and validate checksum
    Packet packet;
    packet._data.assign(data.begin(), data.end());

    if (packet._data.back() != packet._calculateChecksum())
    {
        throw std::invalid_argument("Invalid packet checksum");
    }

    return packet;
}

std::span<const uint8_t> Packet::getParameters() const
{
    if (_data.size() <= 5)  // No parameters if only header + id + length + instruction + checksum
    {
        return {};
    }
    return std::span<const uint8_t>(_data.data() + 5, _data.size() - 6);  // Exclude checksum
}

st3215::error_t Packet::getError() const
{
    if (!isStatusPacket())
    {
        throw std::runtime_error("Not a status packet");
    }
    return static_cast<st3215::error_t>(getParameters()[0]);
}

bool Packet::isStatusPacket() const
{
    return _data.size() >= 5 && (_data[4] & 0x80);  // Bit 7 set in instruction byte
}

bool Packet::isValid() const
{
    return _data.size() >= MIN_PACKET_SIZE &&
           _data[0] == PACKET_HEADER[0] &&
           _data[1] == PACKET_HEADER[1] &&
           _data[3] == _data.size() - 4 &&  // Length check
           _data.back() == _calculateChecksum();
}

uint8_t Packet::_calculateChecksum() const
{
    if (_data.size() < 3)  // Need at least ID byte to calculate checksum
    {
        return 0;
    }

    // Sum everything except header and checksum
    const uint8_t sum = std::accumulate(
        _data.begin() + 2,  // Start after header
        _data.end() - 1,    // Stop before checksum
        0u);                // Initial value

    return ~sum;  // Return inverted sum
}

StatusPacket StatusPacket::parse(std::span<const uint8_t> data)
{
    auto packet = Packet::parse(data);
    if (!packet.isStatusPacket())
    {
        throw std::invalid_argument("Not a status packet");
    }
    return StatusPacket(std::move(packet));
}

std::span<const uint8_t> StatusPacket::getParameterData() const
{
    auto params = getParameters();
    return params.subspan(1);  // Skip error byte
}
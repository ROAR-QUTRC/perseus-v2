// perseus-arm-teleop-mini.cpp
#include "perseus-arm-teleop-mini.hpp"

#include <thread>

ST3215ServoReaderMini::ST3215ServoReaderMini(const std::string& port, unsigned int baud_rate)
    : _serial_port(_io_service, port)
{
    _serial_port.set_option(boost::asio::serial_port::baud_rate(baud_rate));
    _serial_port.set_option(boost::asio::serial_port::character_size(8));
    _serial_port.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    _serial_port.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

ST3215ServoReaderMini::~ST3215ServoReaderMini()
{
    if (_serial_port.is_open())
        _serial_port.close();
}

uint16_t ST3215ServoReaderMini::readPosition(uint8_t servo_id)
{
    auto cmd = _createReadCommand(servo_id, 0x38, 2);
    boost::asio::write(_serial_port, boost::asio::buffer(cmd));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::array<uint8_t, 8> response;
    boost::asio::read(_serial_port, boost::asio::buffer(response));
    return response[5] | (response[6] << 8);
}

void ST3215ServoReaderMini::writePosition(uint8_t servo_id, uint16_t position)
{
    std::vector<uint8_t> data = {static_cast<uint8_t>(position & 0xFF),
                                 static_cast<uint8_t>((position >> 8) & 0xFF)};
    auto cmd = _createWriteCommand(servo_id, 0x2A, data);
    boost::asio::write(_serial_port, boost::asio::buffer(cmd));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

std::vector<uint8_t> ST3215ServoReaderMini::_createReadCommand(uint8_t id, uint8_t address, uint8_t size)
{
    std::vector<uint8_t> cmd = {0xFF, 0xFF, id, 0x04, 0x02, address, size, 0x00};
    uint8_t checksum = 0;
    for (size_t i = 2; i < cmd.size() - 1; ++i) checksum += cmd[i];
    cmd[7] = ~checksum;
    return cmd;
}

std::vector<uint8_t> ST3215ServoReaderMini::_createWriteCommand(uint8_t id, uint8_t address, const std::vector<uint8_t>& data)
{
    std::vector<uint8_t> cmd = {0xFF, 0xFF, id, static_cast<uint8_t>(data.size() + 3), 0x03, address};
    cmd.insert(cmd.end(), data.begin(), data.end());
    uint8_t checksum = 0;
    for (size_t i = 2; i < cmd.size(); ++i) checksum += cmd[i];
    cmd.push_back(~checksum);
    return cmd;
}
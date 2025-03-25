// perseus-arm-teleop-mini.hpp
#pragma once
#include <boost/asio.hpp>
#include <vector>

class ST3215ServoReaderMini
{
public:
    ST3215ServoReaderMini(const std::string& port, unsigned int baud_rate);
    ~ST3215ServoReaderMini();
    uint16_t readPosition(uint8_t servo_id);
    void writePosition(uint8_t servo_id, uint16_t position);

private:
    boost::asio::io_service _io_service;
    boost::asio::serial_port _serial_port;
    std::vector<uint8_t> _createReadCommand(uint8_t id, uint8_t address, uint8_t size);
    std::vector<uint8_t> _createWriteCommand(uint8_t id, uint8_t address, const std::vector<uint8_t>& data);
};
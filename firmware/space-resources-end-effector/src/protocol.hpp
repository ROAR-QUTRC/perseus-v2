#pragma once

#include <cstdint>

namespace protocol {

// Frame markers
constexpr uint8_t COMMAND_MARKER = 0xAA;
constexpr uint8_t TELEMETRY_MARKER = 0xBB;

// Command IDs
constexpr uint8_t CMD_SET_SERVO = 0x01;
constexpr uint8_t CMD_SET_HEATER = 0x02;
constexpr uint8_t CMD_SET_ALL = 0x03;
constexpr uint8_t CMD_GET_STATUS = 0x04;
constexpr uint8_t CMD_STOP_ALL = 0xFE;
constexpr uint8_t CMD_HEARTBEAT = 0xFF;

// Telemetry ID
constexpr uint8_t TELEMETRY_ID = 0x80;

// Error flag bits
constexpr uint8_t ERR_OVERCURRENT = 1 << 0;
constexpr uint8_t ERR_COMM_TIMEOUT = 1 << 1;
constexpr uint8_t ERR_SERVO_FAULT = 1 << 2;
constexpr uint8_t ERR_HEATER_FAULT = 1 << 3;
constexpr uint8_t ERR_TRANSCEIVER_FAULT = 1 << 4;

// Telemetry frame size (excluding CRC)
constexpr uint8_t TELEMETRY_FRAME_SIZE = 10;

/// CRC-8/SMBUS: polynomial 0x07, init 0x00
uint8_t crc8(const uint8_t* data, uint8_t length);

/// Decoded command from Orin
struct Command {
    uint8_t type;
    uint8_t seq;
    int16_t servo_speed;   // -1000 to +1000
    uint8_t heater_duty;   // 0 to 255
    bool valid;
};

/// Decode a received command frame
Command decode_command(const uint8_t* data, uint8_t length);

/// Encode a telemetry frame into buffer. Returns frame length.
uint8_t encode_telemetry(uint8_t* buf, uint8_t seq, uint16_t current_ma,
                         int16_t servo_speed, uint8_t heater_duty,
                         uint8_t error_flags);

}  // namespace protocol

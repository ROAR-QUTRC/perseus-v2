#include "protocol.hpp"

namespace protocol
{

    uint8_t crc8(const uint8_t* data, uint8_t length)
    {
        uint8_t crc = 0x00;
        for (uint8_t i = 0; i < length; i++)
        {
            crc ^= data[i];
            for (uint8_t bit = 0; bit < 8; bit++)
            {
                if (crc & 0x80)
                {
                    crc = (crc << 1) ^ 0x07;
                }
                else
                {
                    crc = crc << 1;
                }
            }
        }
        return crc;
    }

    Command decode_command(const uint8_t* data, uint8_t length)
    {
        Command cmd = {};
        cmd.valid = false;

        if (length < 4)
        {
            return cmd;
        }
        if (data[0] != COMMAND_MARKER)
        {
            return cmd;
        }

        // Verify CRC
        uint8_t expected_crc = crc8(data, length - 1);
        if (expected_crc != data[length - 1])
        {
            return cmd;
        }

        cmd.type = data[1];
        cmd.seq = data[2];
        cmd.servo_speed = 0;
        cmd.heater_duty = 0;

        switch (cmd.type)
        {
        case CMD_SET_SERVO:
            if (length == 6)
            {
                cmd.servo_speed =
                    static_cast<int16_t>((data[3] << 8) | data[4]);
                cmd.valid = true;
            }
            break;

        case CMD_SET_HEATER:
            if (length == 5)
            {
                cmd.heater_duty = data[3];
                cmd.valid = true;
            }
            break;

        case CMD_SET_ALL:
            if (length == 7)
            {
                cmd.servo_speed =
                    static_cast<int16_t>((data[3] << 8) | data[4]);
                cmd.heater_duty = data[5];
                cmd.valid = true;
            }
            break;

        case CMD_GET_STATUS:
        case CMD_STOP_ALL:
        case CMD_HEARTBEAT:
            if (length == 4)
            {
                cmd.valid = true;
            }
            break;

        default:
            break;
        }

        return cmd;
    }

    uint8_t encode_telemetry(uint8_t* buf, uint8_t seq, uint16_t current_ma,
                             int16_t servo_speed, uint8_t heater_duty,
                             uint8_t error_flags)
    {
        buf[0] = TELEMETRY_MARKER;
        buf[1] = TELEMETRY_ID;
        buf[2] = seq;
        buf[3] = (current_ma >> 8) & 0xFF;
        buf[4] = current_ma & 0xFF;
        buf[5] = (static_cast<uint16_t>(servo_speed) >> 8) & 0xFF;
        buf[6] = static_cast<uint16_t>(servo_speed) & 0xFF;
        buf[7] = heater_duty;
        buf[8] = error_flags;
        buf[9] = crc8(buf, 9);
        return TELEMETRY_FRAME_SIZE;
    }

}  // namespace protocol

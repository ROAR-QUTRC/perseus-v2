"""Binary protocol for 915MHz transceiver communication.

Shared protocol between Orin (Python) and Pico (C++).
Max payload: 61 bytes (RFM69 limit).

Command frame (Orin -> Pico):
    [0xAA] [CMD_TYPE] [SEQ] [payload...] [CRC8]

Telemetry frame (Pico -> Orin):
    [0xBB] [0x80] [SEQ] [current_ma:u16] [servo_speed:i16]
    [heater_duty:u8] [error_flags:u8] [CRC8]
"""

import struct

# Frame markers
COMMAND_MARKER = 0xAA
TELEMETRY_MARKER = 0xBB

# Command IDs
CMD_SET_SERVO = 0x01
CMD_SET_HEATER = 0x02
CMD_SET_ALL = 0x03
CMD_GET_STATUS = 0x04
CMD_STOP_ALL = 0xFE
CMD_HEARTBEAT = 0xFF

# Telemetry ID
TELEMETRY_ID = 0x80

# Error flag bits
ERR_OVERCURRENT = 1 << 0
ERR_COMM_TIMEOUT = 1 << 1
ERR_SERVO_FAULT = 1 << 2
ERR_HEATER_FAULT = 1 << 3
ERR_TRANSCEIVER_FAULT = 1 << 4


def crc8(data: bytes) -> int:
    """CRC-8/SMBUS: polynomial 0x07, init 0x00."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc = crc << 1
            crc &= 0xFF
    return crc


def encode_set_servo(seq: int, speed: float) -> bytes:
    """Encode SET_SERVO command. speed: -1.0 to +1.0."""
    speed_i16 = int(max(-1.0, min(1.0, speed)) * 1000)
    payload = struct.pack(">BBBh", COMMAND_MARKER, CMD_SET_SERVO, seq & 0xFF, speed_i16)
    return payload + bytes([crc8(payload)])


def encode_set_heater(seq: int, duty: float) -> bytes:
    """Encode SET_HEATER command. duty: 0.0 to 1.0."""
    duty_u8 = int(max(0.0, min(1.0, duty)) * 255)
    payload = struct.pack(">BBBB", COMMAND_MARKER, CMD_SET_HEATER, seq & 0xFF, duty_u8)
    return payload + bytes([crc8(payload)])


def encode_set_all(seq: int, speed: float, duty: float) -> bytes:
    """Encode SET_ALL command. speed: -1.0..+1.0, duty: 0.0..1.0."""
    speed_i16 = int(max(-1.0, min(1.0, speed)) * 1000)
    duty_u8 = int(max(0.0, min(1.0, duty)) * 255)
    payload = struct.pack(
        ">BBBhB", COMMAND_MARKER, CMD_SET_ALL, seq & 0xFF, speed_i16, duty_u8
    )
    return payload + bytes([crc8(payload)])


def encode_get_status(seq: int) -> bytes:
    """Encode GET_STATUS command."""
    payload = struct.pack(">BBB", COMMAND_MARKER, CMD_GET_STATUS, seq & 0xFF)
    return payload + bytes([crc8(payload)])


def encode_stop_all(seq: int) -> bytes:
    """Encode STOP_ALL command."""
    payload = struct.pack(">BBB", COMMAND_MARKER, CMD_STOP_ALL, seq & 0xFF)
    return payload + bytes([crc8(payload)])


def encode_heartbeat(seq: int) -> bytes:
    """Encode HEARTBEAT command."""
    payload = struct.pack(">BBB", COMMAND_MARKER, CMD_HEARTBEAT, seq & 0xFF)
    return payload + bytes([crc8(payload)])


class Telemetry:
    """Decoded telemetry frame from the Pico."""

    __slots__ = (
        "seq",
        "current_amps",
        "servo_speed",
        "heater_duty",
        "overcurrent",
        "comm_timeout",
        "servo_fault",
        "heater_fault",
        "transceiver_fault",
    )

    def __init__(
        self,
        seq: int,
        current_ma: int,
        servo_speed_i16: int,
        heater_duty_u8: int,
        error_flags: int,
    ):
        self.seq = seq
        self.current_amps = current_ma / 1000.0
        self.servo_speed = servo_speed_i16 / 1000.0
        self.heater_duty = heater_duty_u8 / 255.0
        self.overcurrent = bool(error_flags & ERR_OVERCURRENT)
        self.comm_timeout = bool(error_flags & ERR_COMM_TIMEOUT)
        self.servo_fault = bool(error_flags & ERR_SERVO_FAULT)
        self.heater_fault = bool(error_flags & ERR_HEATER_FAULT)
        self.transceiver_fault = bool(error_flags & ERR_TRANSCEIVER_FAULT)


def decode_telemetry(data: bytes) -> Telemetry | None:
    """Decode a telemetry frame. Returns None if invalid."""
    if len(data) < 9:
        return None
    if data[0] != TELEMETRY_MARKER:
        return None
    if data[1] != TELEMETRY_ID:
        return None

    # Verify CRC over all bytes except the last
    if crc8(data[:-1]) != data[-1]:
        return None

    seq = data[2]
    current_ma, servo_speed_i16, heater_duty_u8, error_flags = struct.unpack(
        ">hhBB", data[3:9]
    )
    # current_ma is unsigned in meaning but packed as signed for struct alignment;
    # reinterpret as unsigned
    current_ma = struct.unpack(">H", data[3:5])[0]

    return Telemetry(seq, current_ma, servo_speed_i16, heater_duty_u8, error_flags)


def encode_telemetry(
    seq: int,
    current_ma: int,
    servo_speed_i16: int,
    heater_duty_u8: int,
    error_flags: int,
) -> bytes:
    """Encode a telemetry frame (used by Pico side, useful for testing)."""
    payload = struct.pack(
        ">BBBHhBB",
        TELEMETRY_MARKER,
        TELEMETRY_ID,
        seq & 0xFF,
        current_ma & 0xFFFF,
        servo_speed_i16,
        heater_duty_u8 & 0xFF,
        error_flags & 0xFF,
    )
    return payload + bytes([crc8(payload)])

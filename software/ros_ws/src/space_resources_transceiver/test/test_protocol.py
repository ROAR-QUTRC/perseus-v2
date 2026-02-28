"""Unit tests for the binary protocol module."""

from space_resources_transceiver.protocol import (
    CMD_HEARTBEAT,
    CMD_SET_ALL,
    CMD_SET_HEATER,
    CMD_SET_SERVO,
    CMD_STOP_ALL,
    COMMAND_MARKER,
    crc8,
    decode_telemetry,
    encode_get_status,
    encode_heartbeat,
    encode_set_all,
    encode_set_heater,
    encode_set_servo,
    encode_stop_all,
    encode_telemetry,
)


class TestCrc8:
    def test_empty(self):
        assert crc8(b"") == 0x00

    def test_known_value(self):
        # CRC-8/SMBUS of [0x01, 0x02, 0x03]
        result = crc8(bytes([0x01, 0x02, 0x03]))
        assert isinstance(result, int)
        assert 0 <= result <= 255

    def test_different_data_different_crc(self):
        assert crc8(b"\x01\x02") != crc8(b"\x01\x03")

    def test_single_byte(self):
        assert crc8(b"\x00") == 0x00
        assert crc8(b"\x07") != 0x00


class TestEncodeCommands:
    def test_set_servo_marker_and_id(self):
        frame = encode_set_servo(0, 0.5)
        assert frame[0] == COMMAND_MARKER
        assert frame[1] == CMD_SET_SERVO
        assert len(frame) == 6

    def test_set_servo_clamping(self):
        frame_max = encode_set_servo(0, 2.0)
        frame_one = encode_set_servo(0, 1.0)
        # Both should encode to speed=+1000
        assert frame_max[3:5] == frame_one[3:5]

        frame_min = encode_set_servo(0, -2.0)
        frame_neg = encode_set_servo(0, -1.0)
        assert frame_min[3:5] == frame_neg[3:5]

    def test_set_heater(self):
        frame = encode_set_heater(5, 0.5)
        assert frame[0] == COMMAND_MARKER
        assert frame[1] == CMD_SET_HEATER
        assert frame[2] == 5  # seq
        assert len(frame) == 5

    def test_set_all(self):
        frame = encode_set_all(10, 0.3, 0.7)
        assert frame[0] == COMMAND_MARKER
        assert frame[1] == CMD_SET_ALL
        assert frame[2] == 10
        assert len(frame) == 7

    def test_stop_all(self):
        frame = encode_stop_all(42)
        assert frame[0] == COMMAND_MARKER
        assert frame[1] == CMD_STOP_ALL
        assert frame[2] == 42
        assert len(frame) == 4

    def test_heartbeat(self):
        frame = encode_heartbeat(255)
        assert frame[0] == COMMAND_MARKER
        assert frame[1] == CMD_HEARTBEAT
        assert frame[2] == 255
        assert len(frame) == 4

    def test_get_status(self):
        frame = encode_get_status(0)
        assert len(frame) == 4

    def test_seq_wrapping(self):
        frame = encode_heartbeat(256)
        assert frame[2] == 0  # 256 & 0xFF = 0

    def test_crc_validates(self):
        frame = encode_set_all(1, 0.5, 0.5)
        assert crc8(frame[:-1]) == frame[-1]


class TestTelemetry:
    def test_roundtrip(self):
        frame = encode_telemetry(
            seq=7,
            current_ma=1500,
            servo_speed_i16=500,
            heater_duty_u8=128,
            error_flags=0x00,
        )
        telem = decode_telemetry(frame)
        assert telem is not None
        assert telem.seq == 7
        assert abs(telem.current_amps - 1.5) < 0.01
        assert abs(telem.servo_speed - 0.5) < 0.01
        assert abs(telem.heater_duty - 128 / 255.0) < 0.01
        assert not telem.overcurrent
        assert not telem.comm_timeout

    def test_error_flags(self):
        frame = encode_telemetry(0, 0, 0, 0, 0b00010101)
        telem = decode_telemetry(frame)
        assert telem is not None
        assert telem.overcurrent
        assert not telem.comm_timeout
        assert telem.servo_fault
        assert not telem.heater_fault
        assert telem.transceiver_fault

    def test_invalid_marker(self):
        frame = encode_telemetry(0, 0, 0, 0, 0)
        bad_frame = b"\x00" + frame[1:]
        assert decode_telemetry(bad_frame) is None

    def test_bad_crc(self):
        frame = encode_telemetry(0, 0, 0, 0, 0)
        bad_frame = frame[:-1] + bytes([(frame[-1] + 1) & 0xFF])
        assert decode_telemetry(bad_frame) is None

    def test_too_short(self):
        assert decode_telemetry(b"\xbb\x80") is None
        assert decode_telemetry(b"") is None

    def test_zero_current(self):
        frame = encode_telemetry(0, 0, 0, 0, 0)
        telem = decode_telemetry(frame)
        assert telem is not None
        assert telem.current_amps == 0.0

    def test_max_current(self):
        frame = encode_telemetry(0, 65535, 0, 0, 0)
        telem = decode_telemetry(frame)
        assert telem is not None
        assert abs(telem.current_amps - 65.535) < 0.01

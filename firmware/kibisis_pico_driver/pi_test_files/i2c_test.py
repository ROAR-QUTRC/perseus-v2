#!/usr/bin/env python3
"""
kibisis_pi5_test.py
Pi 5 I2C master test script for the Kibisis Pico Driver.
Writes motor speeds and reads back encoder counts.
"""

import smbus2
import struct
import time

SLAVE_ADDR = 0x41
BUS_NUM    = 1

# Register map (must match i2c_slave.hpp)
REG_MOTOR_A_SPEED = 0x00
REG_MOTOR_B_SPEED = 0x01
REG_ENC_A         = 0x10
REG_ENC_B         = 0x14
REG_STATUS        = 0x20

bus = smbus2.SMBus(BUS_NUM)


def scan_bus():
    print("Scanning I2C bus...")
    found = []
    for addr in range(0x08, 0x78):
        try:
            bus.read_byte(addr)
            print(f"  Found device at 0x{addr:02X}")
            found.append(addr)
        except OSError:
            pass
    if not found:
        print("  No devices found.")
    return found


def set_motors(speed_a: int, speed_b: int):
    """Set motor speeds. Range: -100 to 100."""
    speed_a = max(-100, min(100, speed_a))
    speed_b = max(-100, min(100, speed_b))
    bus.write_i2c_block_data(SLAVE_ADDR, REG_MOTOR_A_SPEED, [
        speed_a & 0xFF,
        speed_b & 0xFF,
    ])


def read_encoders() -> tuple[int, int]:
    """Read both encoder counts as signed 32-bit integers."""
    raw = bus.read_i2c_block_data(SLAVE_ADDR, REG_ENC_A, 8)
    enc_a = struct.unpack_from('<i', bytes(raw), 0)[0]
    enc_b = struct.unpack_from('<i', bytes(raw), 4)[0]
    return enc_a, enc_b


def read_status() -> int:
    return bus.read_byte_data(SLAVE_ADDR, REG_STATUS)


if __name__ == "__main__":
    scan_bus()

    print(f"\nConnecting to Kibisis Pico Driver at 0x{SLAVE_ADDR:02X}...")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            set_motors(50, -50)
            enc_a, enc_b = read_encoders()
            status       = read_status()
            print(f"Status: 0x{status:02X}  Enc A: {enc_a:8d}  Enc B: {enc_b:8d}")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping motors...")
        set_motors(0, 0)
        bus.close()
        print("Done.")

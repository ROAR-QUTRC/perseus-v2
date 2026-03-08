#!/usr/bin/env python3
"""
kibisis_motor_test.py
Pi 5 motor test script for the Kibisis Pico Driver.
Tests motor A and B independently with a series of speed/direction commands.
"""

import smbus2
import time

SLAVE_ADDR = 0x41
BUS_NUM    = 1

# Register map (must match i2c_slave.hpp)
REG_MOTOR_A_SPEED = 0x00
REG_MOTOR_B_SPEED = 0x01

bus = smbus2.SMBus(BUS_NUM)


def set_motor_a(speed: int):
    """Set motor A speed. Range: -100 to 100."""
    speed = max(-100, min(100, speed))
    bus.write_byte_data(SLAVE_ADDR, REG_MOTOR_A_SPEED, speed & 0xFF)


def set_motor_b(speed: int):
    """Set motor B speed. Range: -100 to 100."""
    speed = max(-100, min(100, speed))
    bus.write_byte_data(SLAVE_ADDR, REG_MOTOR_B_SPEED, speed & 0xFF)


def set_motors(speed_a: int, speed_b: int):
    """Set both motor speeds in one transaction."""
    speed_a = max(-100, min(100, speed_a))
    speed_b = max(-100, min(100, speed_b))
    bus.write_i2c_block_data(SLAVE_ADDR, REG_MOTOR_A_SPEED, [
        speed_a & 0xFF,
        speed_b & 0xFF,
    ])


def stop():
    set_motors(0, 0)
    print("  >> Motors stopped")


def run_test(label: str, speed_a: int, speed_b: int, duration: float):
    print(f"  {label}: A={speed_a:+4d}%  B={speed_b:+4d}%  ({duration}s)")
    set_motors(speed_a, speed_b)
    time.sleep(duration)


if __name__ == "__main__":
    print("Kibisis Motor Test")
    print("==================")
    print(f"Connecting to 0x{SLAVE_ADDR:02X} on bus {BUS_NUM}...\n")

    try:
        bus.read_byte(SLAVE_ADDR)
    except OSError:
        print(f"ERROR: No device found at 0x{SLAVE_ADDR:02X}. Check wiring and flash.")
        exit(1)

    print("Device found. Starting tests...\n")

    try:
        # --- Motor A only ---
        print("[ Motor A ]")
        run_test("Forward  25%",  25,   0, 2.0)
        run_test("Forward  50%",  50,   0, 2.0)
        run_test("Forward 100%", 100,   0, 2.0)
        stop(); time.sleep(0.5)
        run_test("Reverse  25%",  -25,  0, 2.0)
        run_test("Reverse  50%",  -50,  0, 2.0)
        run_test("Reverse 100%", -100,  0, 2.0)
        stop(); time.sleep(0.5)

        # --- Motor B only ---
        print("\n[ Motor B ]")
        run_test("Forward  25%",   0,  25, 2.0)
        run_test("Forward  50%",   0,  50, 2.0)
        run_test("Forward 100%",   0, 100, 2.0)
        stop(); time.sleep(0.5)
        run_test("Reverse  25%",   0, -25, 2.0)
        run_test("Reverse  50%",   0, -50, 2.0)
        run_test("Reverse 100%",   0, -100, 2.0)
        stop(); time.sleep(0.5)

        # --- Both motors ---
        print("\n[ Both Motors ]")
        run_test("Forward together",   50,  50, 2.0)
        run_test("Reverse together",  -50, -50, 2.0)
        run_test("Spin left  (A fwd, B rev)",  50, -50, 2.0)
        run_test("Spin right (A rev, B fwd)", -50,  50, 2.0)
        stop(); time.sleep(0.5)

        # --- Direction reversal (tests brake logic) ---
        print("\n[ Direction Reversal - tests brake ]")
        run_test("Full forward", 100, 100, 2.0)
        print("  >> Reversing direction (brake should fire on Pico)...")
        run_test("Full reverse", -100, -100, 2.0)
        stop()

        print("\nAll tests complete.")

    except KeyboardInterrupt:
        print("\nTest interrupted.")

    finally:
        stop()
        bus.close()
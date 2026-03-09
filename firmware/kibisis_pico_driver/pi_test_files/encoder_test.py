#!/usr/bin/env python3
"""
kibisis_encoder_test.py

Tests quadrature encoder reads from the Kibisis Pico Driver over I2C.

Encoder register layout (little-endian int32_t, 4 bytes each):
  0x10..0x13  Encoder A count
  0x14..0x17  Encoder B count

Wiring:
  Encoder A: Yellow=GP13 (ChA), White=GP14 (ChB), Green=GND, Blue=Vcc
  Encoder B: Yellow=GP15 (ChA), White=GP16 (ChB), Green=GND, Blue=Vcc

Motor spec: 64 CPR * 4 edges * 100:1 = 6400 counts/rev output shaft
"""

import smbus2
import struct
import time

BUS_NUM    = 1
SLAVE_ADDR = 0x41

# Register addresses
REG_ENC_A = 0x10  # int32_t, 4 bytes, little-endian
REG_ENC_B = 0x14  # int32_t, 4 bytes, little-endian
REG_STATUS = 0x20

COUNTS_PER_REV = 6400  # 64 CPR * 4 edges * 100:1

def read_encoder(bus, reg):
    """Read a 4-byte little-endian int32_t from the given register address."""
    data = bus.read_i2c_block_data(SLAVE_ADDR, reg, 4)
    return struct.unpack('<i', bytes(data))[0]

def read_status(bus):
    data = bus.read_i2c_block_data(SLAVE_ADDR, REG_STATUS, 1)
    return data[0]

def counts_to_revs(counts):
    return counts / COUNTS_PER_REV

def main():
    print(f"Kibisis encoder test | I2C bus {BUS_NUM}, address 0x{SLAVE_ADDR:02X}")
    print(f"Resolution: {COUNTS_PER_REV} counts/rev (64 CPR × 4 edges × 100:1)")
    print("-" * 60)

    bus = smbus2.SMBus(BUS_NUM)
    prev_a = None
    prev_b = None
    prev_time = None

    try:
        while True:
            now = time.monotonic()
            enc_a = read_encoder(bus, REG_ENC_A)
            enc_b = read_encoder(bus, REG_ENC_B)
            status = read_status(bus)

            revs_a = counts_to_revs(enc_a)
            revs_b = counts_to_revs(enc_b)

            # Calculate velocity (counts/sec → RPM) if we have a previous reading
            rpm_a = 0.0
            rpm_b = 0.0
            if prev_a is not None and prev_time is not None:
                dt = now - prev_time
                if dt > 0:
                    rpm_a = ((enc_a - prev_a) / COUNTS_PER_REV) / dt * 60.0
                    rpm_b = ((enc_b - prev_b) / COUNTS_PER_REV) / dt * 60.0

            prev_a, prev_b, prev_time = enc_a, enc_b, now

            print(
                f"Enc A: {enc_a:>10d} counts  {revs_a:>8.3f} rev  {rpm_a:>7.1f} RPM  |  "
                f"Enc B: {enc_b:>10d} counts  {revs_b:>8.3f} rev  {rpm_b:>7.1f} RPM  |  "
                f"Status: 0x{status:02X}"
            )

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        bus.close()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
kibisis_moisture_test.py

Triggers and reads the moisture sensor on the Kibisis Pico Driver over I2C.

Flow:
  1. Pi writes 0x01 to MOISTURE_SAMPLE (0x30) to request a sample
  2. Pico takes the ADC reading and clears the register
  3. Pi reads the 2-byte result from MOISTURE_VALUE (0x31)

Register layout:
  0x30  MOISTURE_SAMPLE   write 0x01 to trigger, Pico clears to 0 when done
  0x31  MOISTURE_VALUE_0  low byte  } uint16_t little-endian, 0-4095
  0x32  MOISTURE_VALUE_1  high byte }
"""

import smbus2
import struct
import time

BUS_NUM    = 1
SLAVE_ADDR = 0x41

REG_MOISTURE_SAMPLE = 0x30
REG_MOISTURE_VALUE  = 0x31

def trigger_sample(bus):
    bus.write_byte_data(SLAVE_ADDR, REG_MOISTURE_SAMPLE, 0x01)

def read_moisture(bus):
    data = bus.read_i2c_block_data(SLAVE_ADDR, REG_MOISTURE_VALUE, 2)
    return struct.unpack('<H', bytes(data))[0]

def sample_and_read(bus, settle_ms=20):
    """Trigger a sample, wait for Pico to process it, then read the result."""
    trigger_sample(bus)
    time.sleep(settle_ms / 1000.0)  # give Pico time to sample and clear register
    return read_moisture(bus)

def main():
    print(f"Kibisis moisture sensor test | I2C bus {BUS_NUM}, address 0x{SLAVE_ADDR:02X}")
    print(f"Range: 0 (dry) – 4095 (wet)")
    print("-" * 40)

    bus = smbus2.SMBus(BUS_NUM)
    try:
        while True:
            value = sample_and_read(bus)
            bar = "#" * (value * 40 // 4095)
            print(f"Moisture: {value:>4d} / 4095  |{bar:<40}|")
            time.sleep(1.0)  # sample at 1Hz - no need to read continuously
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        bus.close()

if __name__ == "__main__":
    main()


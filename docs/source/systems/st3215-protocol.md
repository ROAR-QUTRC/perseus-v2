# ST3215 Servo Protocol Documentation

## Overview

The ST3215 servo uses a serial communication protocol that allows control and monitoring of various servo parameters. This protocol is part of the Feetech servo family and follows the STS series specification.

**Reference:** For complete protocol details, refer to the [Feetech STS3215 Servo User Manual](https://www.feetechrc.com/service-programmable-servo.html).

## Memory Map

### EPROM Memory Tables

#### Read-Only Registers

| Address | Name                       | Size (bytes) | Range | Description                   |
| ------- | -------------------------- | ------------ | ----- | ----------------------------- |
| 0       | Firmware Main Version      | 1            | -     | Read-only firmware version    |
| 1       | Firmware Secondary Version | 1            | -     | Read-only firmware subversion |
| 3       | Servo Main Version         | 1            | -     | Hardware main version         |
| 4       | Servo Sub Version          | 1            | -     | Hardware subversion           |

#### Read/Write Registers

| Address | Name                  | Size (bytes) | Range      | Default | Units/Notes        |
| ------- | --------------------- | ------------ | ---------- | ------- | ------------------ |
| 5       | ID                    | 1            | 0-253      | 1       | -                  |
| 6       | Baud Rate             | 1            | 0-7        | 4       | Index value        |
| 7       | Return Delay Time     | 1            | 0-254      | 250     | 2μs per unit       |
| 8       | Status Return Level   | 1            | 0-1        | 1       | -                  |
| 9-10    | Min Position Limit    | 2            | 0-1023     | 0       | Encoder steps      |
| 11-12   | Max Position Limit    | 2            | 0-1023     | 1023    | Encoder steps      |
| 13      | Max Temperature Limit | 1            | 0-100      | 80      | °C                 |
| 14      | Max Input Voltage     | 1            | 0-254      | 140     | 0.1V units (14.0V) |
| 15      | Min Input Voltage     | 1            | 0-254      | 80      | 0.1V units (8.0V)  |
| 16-17   | Max Torque Limit      | 2            | 0-1000     | 1000    | 0.1% of max torque |
| 26      | CW Dead Band          | 1            | 0-32       | 2       | Encoder steps      |
| 27      | CCW Dead Band         | 1            | 0-32       | 2       | Encoder steps      |
| 31-32   | Position Offset       | 2            | -2047-2047 | 0       | Encoder steps      |
| 33      | Operating Mode        | 1            | 0-3        | 0       | Mode index         |

### SRAM Memory Tables

#### Read/Write Registers

| Address | Name          | Size (bytes) | Range        | Default | Units/Notes          |
| ------- | ------------- | ------------ | ------------ | ------- | -------------------- |
| 40      | Torque Enable | 1            | 0-2          | 0       | 0=off, 1=on, 2=hold  |
| 41      | Acceleration  | 1            | 0-254        | 0       | Steps/s² (approx)    |
| 42-43   | Goal Position | 2            | 0-1023       | -       | Encoder steps        |
| 44-45   | Running Time  | 2            | -32766-32766 | 0       | ms                   |
| 46-47   | Goal Speed    | 2            | -1000-1000   | 0       | RPM (approx)         |
| 48-49   | Torque Limit  | 2            | 0-1000       | 1000    | 0.1% of max torque   |
| 55      | Lock          | 1            | 0-1          | 1       | 0=unlocked, 1=locked |

#### Read-Only Status Registers

| Address | Name                | Size (bytes) | Range        | Description                                |
| ------- | ------------------- | ------------ | ------------ | ------------------------------------------ |
| 56-57   | Present Position    | 2            | 0-1023       | Current servo position (encoder steps)     |
| 58-59   | Present Speed       | 2            | -32768-32767 | Current speed (RPM, approx)                |
| 60-61   | Present Load        | 2            | -1000-1000   | Current load (0.1% of max torque)          |
| 62      | Present Voltage     | 1            | -            | Current voltage (0.1V units)               |
| 63      | Present Temperature | 1            | -            | Current temperature (°C)                   |
| 66      | Moving Status       | 1            | 0-1          | Movement status flag (0=stopped, 1=moving) |
| 69-70   | Present Current     | 2            | -            | Current draw (mA)                          |

## Communication Protocol

### Packet Structure

1. Header: 0xFF 0xFF
2. ID: Single byte (0-253, 0xFE for broadcast)
3. Length: Number of parameters + 2
4. Instruction: Command byte
5. Parameters: Variable length
6. Checksum: ~(ID + Length + Instruction + Parameters)

### Instructions

- PING (0x01): Check if servo exists
- READ (0x02): Read from memory
- WRITE (0x03): Write to memory
- REG_WRITE (0x04): Write to register (pending)
- ACTION (0x05): Execute pending REG_WRITE
- SYNC_WRITE (0x83): Write to multiple servos

## Operation Modes

### Position Control

- 16-bit position value (0-1023)
- Supports negative positions using bit 15
- Can specify speed and acceleration
- Position limits can be set in EPROM

### Speed Control (Wheel Mode)

- 16-bit speed value
- Sign bit (bit 15) determines direction
- Acceleration parameter for smooth transitions

### Open Loop Control

- Direct PWM control
- Used for manual torque/speed control
- No position feedback

## Status Monitoring

- Real-time feedback of:
  - Position (0-1023)
  - Speed (-32768 to 32767)
  - Load (0-1000, percentage of max torque)
  - Voltage (0.1V resolution)
  - Temperature (°C)
  - Moving status
  - Current draw

## Error Handling

- Hardware status reporting
- Protection features:
  - Temperature limit
  - Voltage limits
  - Current limit
  - Load protection
  - Dead band protection

## Implementation Notes

- Default baud rate setting: 4
- Status return level configurable
- EPROM can be locked to prevent accidental changes
- Temperature limit default: 80°C
- Voltage operating range: 8-14V

## Protocol Examples

### Reading Position (ID #3)

**TX Packet:**

| Byte | Field       | Hex Value | Description                         |
| ---- | ----------- | --------- | ----------------------------------- |
| 0-1  | Header      | 0xFF 0xFF | Start of packet                     |
| 2    | ID          | 0x03      | Servo #3                            |
| 3    | Length      | 0x04      | 4 bytes following                   |
| 4    | Instruction | 0x02      | READ command                        |
| 5    | Address     | 0x38      | Position register (address 56)      |
| 6    | Size        | 0x01      | Read 2 bytes                        |
| 7    | Checksum    | 0xB8      | ~(0x03 + 0x04 + 0x02 + 0x38 + 0x01) |

**RX Packet:**

| Byte | Field      | Hex Value | Description           |
| ---- | ---------- | --------- | --------------------- |
| 0-1  | Header     | 0xFF 0xFF | Start of packet       |
| 2    | ID         | 0x03      | Servo #3              |
| 3    | Length     | 0x04      | 4 bytes following     |
| 4    | Error      | 0x00      | No error              |
| 5    | Position L | 0xE8      | Low byte of position  |
| 6    | Position H | 0x03      | High byte of position |
| 7    | Checksum   | 0x0E      | Calculated checksum   |

Response shows position 0x03E8 (1000 in decimal).

### Writing Speed (ID #4)

**TX Packet:**

| Byte | Field       | Hex Value | Description                                |
| ---- | ----------- | --------- | ------------------------------------------ |
| 0-1  | Header      | 0xFF 0xFF | Start of packet                            |
| 2    | ID          | 0x04      | Servo #4                                   |
| 3    | Length      | 0x05      | 5 bytes following                          |
| 4    | Instruction | 0x03      | WRITE command                              |
| 5    | Address     | 0x2E      | Speed register (address 46)                |
| 6    | Speed L     | 0x32      | Low byte of speed (50 decimal)             |
| 7    | Speed H     | 0x00      | High byte of speed                         |
| 8    | Checksum    | 0x8E      | ~(0x04 + 0x05 + 0x03 + 0x2E + 0x32 + 0x00) |

**RX Packet:**

| Byte | Field    | Hex Value | Description         |
| ---- | -------- | --------- | ------------------- |
| 0-1  | Header   | 0xFF 0xFF | Start of packet     |
| 2    | ID       | 0x04      | Servo #4            |
| 3    | Length   | 0x02      | 2 bytes following   |
| 4    | Error    | 0x00      | No error - success  |
| 5    | Checksum | 0xF9      | Calculated checksum |

Success response (Error = 0x00).

ST3215 System Hardware for Perseus Lite

## Implementation Notes

ROS2 Jazzy diff_drive_controller requires stamped messages. This code is useful to debug with:

```
ros2 topic pub -r 10 --times 20 /cmd_vel geometry_msgs/msg/TwistStamped "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'base_link'
  },
  twist: {
    linear: {x: 0.5, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.0}
  }
}"
```

## Overview

This ROS 2 hardware interface provides a comprehensive driver for controlling the ST3215 servo motors used in the Perseus Lite robot. The implementation supports precise motor control, state monitoring, and integration with ROS 2 Control framework.
Features

Integration with ROS 2 Control hardware interface
Support for multiple ST3215 servo motors
Real-time velocity and position control
Detailed servo state monitoring including:

- Position
- Velocity
- Load
- Temperature

## Hardware Requirements

Feetech ST3215 Continuous Servo (12-bit encoder, Serial communication interface)

## Configuration

Serial Port Configuration
The hardware interface requires two key parameters in the robot configuration:

serial_port: Path to the serial device (e.g., /dev/ttyACM0)
baud_rate: Communication speed (recommended: 115200)

Servo ID Configuration
Each joint must have a unique id parameter specifying the servo's address.

Example YAML Configuration:
```yaml
serial_port: /dev/ttyACM0
baud_rate: 115200
joints:
  front_left_wheel:
    id: 1
  front_right_wheel:
    id: 2
```

## Communication Protocol

Packet Structure: Header (0xFF 0xFF), ID, Length, Command, Data, Checksum
Supports velocity and position commands
Asynchronous read/write operations
Configurable timeouts for read and write operations

## Error Handling

Includes:

- Comprehensive error logging
- Graceful handling of serial port disconnections
- Safe thread management with RAII principles

## Performance Characteristics

Maximum Velocity: ±1000 RPM
Communication Timeout:

- Read: 10 ms
- Write: 1 ms

## Threading Model

Utilises Boost ASIO for asynchronous I/O.
Dedicated IO thread for non-blocking communication. Thread-safe operations using mutex

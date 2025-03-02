# Perseus Networked Arm Teleoperation

Networked version of the arm teleoperation system for Perseus robot. This system allows controlling a follower arm over a network connection from a leader arm.

## Features

- Leader-follower architecture for networked robotic arm control
- Real-time position, torque, and calibration data synchronization
- Automatic reconnection and error handling
- Support for individual servo mirroring control
- Robust binary communication protocol with checksums
- Intuitive ncurses-based user interface

## Components

The system consists of two main executables:

1. **arm-teleop-leader**: Controls the leader arm and sends commands to the follower
2. **arm-teleop-follower**: Receives commands from the leader and controls the follower arm

## Building

```bash
mkdir build && cd build
cmake ..
make
```

## Usage

### Leader

The leader application connects to the physical leader arm and sends commands to the follower system:

```bash
./arm-teleop-leader [options]
```

Options:

- `-s, --server ADDRESS`: Server address (default: localhost)
- `-p, --port PORT`: Server port (default: 8787)
- `-h, --help`: Show help message

### Follower

The follower application receives commands from the leader and controls the physical follower arm:

```bash
./arm-teleop-follower [options]
```

Options:

- `-b, --bind ADDRESS`: Address to bind to (default: all interfaces)
- `-p, --port PORT`: Port to listen on (default: 8787)
- `-h, --help`: Show help message

## Operation Instructions

1. Start the follower application on the computer connected to the follower arm
2. Start the leader application on the computer connected to the leader arm
3. Use the leader arm to control the follower arm
4. Press 1-6 keys to toggle mirroring for each servo
5. Press 's' to save calibration and send to follower
6. Press 'l' to load calibration
7. Press 't' to toggle torque protection
8. Press Ctrl+C to exit

## Network Protocol

The system uses a custom binary protocol over TCP:

- Servo position updates
- Calibration data synchronization
- Mirroring commands
- Handshake and keepalive messages

## Dependencies

- Boost.Asio
- ncurses
- yaml-cpp
- simple-networking (from Perseus project)

## License

See the LICENSE file for details.

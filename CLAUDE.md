# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Perseus v2 is a lunar rover project with a comprehensive ROS 2-based software stack, built using Nix for reproducible development environments. The codebase includes firmware for multiple microcontrollers, native C++ libraries, ROS 2 packages, and a web-based user interface.

## Build System

This project uses **Nix flakes** rather than traditional build systems for reproducibility and dependency management.

### Environment Setup

```bash
# Enter development environment (sets up all dependencies)
nix develop

# For simulation environment
nix develop .#simulation

# For documentation environment
nix develop .#docs
```

### Key Build Commands

```bash
# Build the default workspace (includes all ROS packages)
nix build

# Build simulation workspace
nix build .#simulation

# Clean build artifacts
nix run .#clean
# or manually:
./software/scripts/clean.sh

# Format code (runs treefmt with configured formatters)
nix fmt
```

## ROS 2 Workspace

### Environment Variables

- ROS_DOMAIN_ID: 51 (development), 42 (production)
- RMW_IMPLEMENTATION: rmw_cyclonedx_cpp (CycloneDDS middleware)
- RCUTILS_COLORIZED_OUTPUT: 1 (colored output)

### Common ROS Commands

```bash
# Launch full rover system
nix run .#perseus
# or: ros2 launch perseus perseus.launch.py

# Launch controller support
nix run .#xbox_controller
# or: ros2 launch input_devices xbox_controller.launch.py

# Build ROS workspace (from software/ros_ws/)
colcon build --symlink-install

# Run specific packages
ros2 launch perseus_lite perseus_lite.launch.py
ros2 launch autonomy perseus_nav_bringup.launch.py
```

### Package Structure

- `perseus/perseus_*`: Main rover packages (full system)
- `perseus_lite/perseus_lite_*`: Simplified rover variant
- `autonomy/`: Navigation and mapping functionality
- `perseus_hardware/`: Hardware interface layers
- `perseus_payloads/`: Arm and bucket control
- `perseus_sensors/`: Sensor drivers (LiDAR, etc.)
- `input_devices/`: Controller interfaces

## Native C++ Libraries (software/shared/)

Shared libraries built with CMake, used across ROS and firmware:

- `hi-can*`: CAN bus communication protocols
- `crc/`: CRC calculation utilities
- `simple-networking/`: Network communication
- `fd-wrapper/`: File descriptor utilities

Build pattern:

```bash
mkdir build && cd build
cmake .. && make
```

## Web UI Development

Located in `software/web_ui/`, built with SvelteKit + TypeScript.

```bash
# Development server (with hot reload)
yarn dev

# Production build
yarn build

# Start production server
yarn start

# Camera server (Pi/embedded)
yarn camera
```

Key components:

- Widget-based dashboard system in `src/lib/widgets/`
- ROS bridge for real-time communication in `src/lib/scripts/ros-bridge.svelte.ts`
- UI components in `src/lib/components/ui/`

## Firmware Development

ESP32-based firmware in `firmware/` using PlatformIO with ESP-IDF:

- `battery-management-system/`: BMS controller
- `elevator-module/`: Elevator control
- `excavation-bucket/`: Bucket operations
- `light-tower/`: Status lighting

## Hardware Requirements

### OpenGL Applications

ROS GUI tools (rviz2, Gazebo) require special handling on non-NixOS systems:

```bash
# Use nixgl wrapper for OpenGL applications
nixgl rviz2
nixgl gz sim
```

### CAN Bus Setup

```bash
# Set up virtual CAN for testing
./software/scripts/vcan-setup.sh
```

## Architecture Notes

### Communication Stack

- **ROS 2**: High-level robot coordination and control
- **CAN Bus**: Real-time motor control and sensor data
- **WebRTC/WebSocket**: Web UI communication
- **hi-can protocol**: Custom CAN communication layer

### System Integration

- ROS 2 packages handle high-level behaviors (navigation, teleoperation)
- Hardware interface packages bridge ROS to CAN bus using hi-can libraries
- Web UI provides operator interface via rosbridge
- Firmware handles low-level motor control and sensor acquisition

### Development vs Production

- Development uses domain ID 51 with additional debugging tools
- Production uses domain ID 42 with minimal overhead
- Nix flake provides both environments with same software versions

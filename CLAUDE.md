# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Perseus-v2 is the main codebase for the ROAR (Remotely Operated Autonomous Rover) robotics project. It contains ROS 2 Jazzy packages for both a full-sized rover (Perseus) and a smaller variant (Perseus Lite), along with an autonomy stack, hardware interfaces, simulation, and web UI.

## Build System

**Primary Build System**: Nix Flakes (not standard colcon)

```bash
# Enter development environment
nix develop                    # Default shell with all tools
nix develop .#simulation       # Simulation environment
nix develop .#docs             # Documentation environment

# Build
nix build -L                   # Build default workspace
nix build -L .#simulation      # Build simulation workspace

# Run applications
nix run .#perseus              # Launch Perseus rover
nix run .#perseus-lite         # Launch Perseus Lite rover
nix run .#generic_controller   # Launch controller

# Format (required before commits)
nix fmt                        # Format entire project via treefmt

# Run checks
nix flake check -L             # Run all validation checks

# Local development alternative (inside nix develop):
cd software/ros_ws
colcon build --symlink-install --packages-skip perseus_can_if perseus_simulation
source install/setup.bash
```

## Key Environment Variables

Set automatically in nix shell:
- `ROS_DOMAIN_ID`: 51 (dev), 42 (production)
- `RMW_IMPLEMENTATION`: rmw_cyclonedds_cpp
- OpenGL apps (rviz2, Gazebo): Use `nixgl rviz2` on non-NixOS systems

## ROS Workspace Structure

Primary packages in `software/ros_ws/src/`:

- **perseus**, **perseus_lite** - Rover bringup packages
- **perseus_hardware**, **perseus_lite_hardware** - Hardware interfaces (ros2_control)
- **autonomy** - SLAM Toolbox, Nav2, EKF, twist_mux
- **perseus_description**, **perseus_lite_description** - URDF/xacro models
- **perseus_input** - Controller/input device handling
- **perseus_interfaces** - Custom message/service definitions
- **perseus_simulation** - Gazebo simulation

## Architecture

```
Applications / Launch files
         │
Autonomy Stack (Nav2, SLAM Toolbox, EKF, twist_mux)
         │
ROS 2 Control (controller_manager, diff_drive_controller)
         │
Hardware Interface (ST3215SystemHw for Lite, CAN for full)
         │
Physical Hardware (servos, LIDAR, IMU)
```

Coordinate frames: `map → odom → base_link → chassis → sensors`

## Code Style

**Naming**:
- Variables/functions: `snake_case`
- Classes: `PascalCase`
- Constants: `SCREAMING_SNAKE_CASE` (prefer `constexpr`)
- Booleans: verb prefix (`is_valid`, `has_data`, `should_exit`)
- Private class members: underscore prefix (`_member`)

**C++ Specifics**:
- Header guards: `#pragma once`
- C++ includes: `<cstdint>` not `<stdint.h>`
- Files: `.cpp`/`.hpp` (not `.cc`/`.h`)
- No raw `new`/`delete` - use smart pointers
- No global variables
- Functions: aim for <40 lines, refactor at 60

**Formatting**:
- C++: clang-format (Google style base, 4-space indent, Allman braces)
- Python: Black + ruff
- All other formats via treefmt

**All compiler/linter warnings must be fixed before merge** (treated as errors).

## Configuration Files

Key autonomy configs in `software/ros_ws/src/autonomy/config/`:
- `slam_toolbox_params.yaml` - SLAM configuration (CUDA-accelerated on Jetson)
- `ekf_params.yaml` - Extended Kalman Filter for odometry fusion
- `twist_mux.yaml` - cmd_vel arbitration from multiple sources

## Documentation

Standards documentation in `docs/source/standards/software/`:
- `general.md` - Overall naming, design, documentation standards
- `cpp.md` - Comprehensive C++ guidelines
- `python.md` - Python (PEP 8 + extensions)

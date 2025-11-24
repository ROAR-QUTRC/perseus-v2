# Quick Start

To teleoperate with an Xbox controller, with Perseus-Lite powered up you will need to run

## Terminal 1

```console
nix shell
ros2 launch perseus_lite perseus_lite.launch.py cmd_vel_topic:=/joy_vel
```

:::{note}
The `cmd_vel_topic:=/joy_vel` parameter remaps the velocity command topic for compatibility with the twist_mux topic multiplexer. This ensures the Xbox controller commands (published to `/joy_vel`) are correctly routed to the base controller.
:::

## Terminal 2

```console
nix run .#xbox_controller
```

# Basic Operation

This guide covers the basic teleoperation of Perseus-Lite using keyboard control. Perseus-Lite is a simplified version of Perseus that provides essential rover functionality with streamlined hardware and software requirements.

## Prerequisites

The following conditions must be met before beginning:

- The laptop runs a Linux distribution with the latest code from the perseus-v2 Github repository:
  ```console
  git checkout main
  git pull
  ```
- Perseus-Lite hardware is connected via USB (typically `/dev/ttyACM0`)
- Perseus-Lite has sufficient battery charge or is powered by a 12V power supply

## Summary

1. Connect Perseus-Lite hardware
2. Build the software using Nix
3. Launch Perseus-Lite system
4. Launch control software
5. Operate Perseus-Lite safely

## Hardware Connection

1. Connect Perseus-Lite's USB cable to your laptop
2. Verify the connection:
   ```console
   ls /dev/ttyACM*
   ```
   You should see `/dev/ttyACM0` or similar device listed

## Software Setup

### Build the Software

Build the Perseus-Lite software using Nix:

```console
cd perseus-v2
nix build
```

### Launch Perseus-Lite System

Launch the Perseus-Lite ROS2 system:

```console
export LC_ALL=C && export LANG=C
nix develop --command bash -c "cd software/ros_ws && colcon build --packages-up-to perseus_lite && source install/setup.bash && ros2 launch perseus_lite perseus_lite.launch.py"
```

:::{note}
The system will automatically configure the 4 ST3215 servos and start the communication thread. You should see messages indicating successful servo configuration.
:::

### Verify System Status

Open a new terminal and verify the system is running:

```console
export LC_ALL=C && export LANG=C
nix develop --command bash -c "source software/ros_ws/install/setup.bash && ros2 topic list"
```

You should see topics including:

- `/cmd_vel` - Command velocity input
- `/joint_states` - Real-time servo positions and velocities
- `/diff_drive_base_controller/odom` - Odometry data

Check joint states to verify servo feedback:

```console
export LC_ALL=C && export LANG=C
nix develop --command bash -c "source software/ros_ws/install/setup.bash && ros2 topic echo /joint_states --once"
```

You should see output similar to this (values will vary based on wheel positions):

```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: ""
name:
  - front_left_wheel_joint
  - front_right_wheel_joint
  - rear_left_wheel_joint
  - rear_right_wheel_joint
position:
  - 0.0
  - 0.0
  - 0.0
  - 0.0
velocity:
  - 0.0
  - 0.0
  - 0.0
  - 0.0
effort:
  - 0.0
  - 0.0
  - 0.0
  - 0.0
---
```

:::{note}
When the robot is stationary, velocities should be near zero. Position values accumulate over time as the wheels rotate. Effort values represent motor load and should be low when not moving.
:::

## Control Software Launch

### Keyboard Control

In a new terminal, launch keyboard teleoperation:

```console
cd perseus-v2
export LC_ALL=C && export LANG=C
nix develop --command bash -c "source software/ros_ws/install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=key_vel"
```

:::{note}
The `stamped:=true` parameter publishes TwistStamped messages, and the `cmd_vel:=key_vel` remapping is required for compatibility with the twist_mux topic multiplexer on the main branch.
:::

#### Keyboard Controls

The keyboard control interface provides the following commands:

- **Movement Controls:**

  - `i` - Move forward
  - `k` - Stop
  - `,` - Move backward
  - `j` - Turn left
  - `l` - Turn right
  - `u` - Forward + left
  - `o` - Forward + right
  - `m` - Backward + left
  - `.` - Backward + right

- **Speed Controls:**
  - `q/z` - Increase/decrease linear speed by 10%
  - `w/x` - Increase/decrease angular speed by 10%

:::{note}
Control messages are only sent when the terminal with keyboard teleoperation has focus and receives keystrokes. Press `k` to stop movement.
:::

### Xbox Controller (Alternative)

If you prefer Xbox controller input:

```console
cd perseus-v2
export LC_ALL=C && export LANG=C
nix develop --command bash -c "source software/ros_ws/install/setup.bash && ros2 launch input_devices xbox_controller.launch.py"
```

## Safe Operation

### Safety Protocol

Before operating Perseus-Lite:

1. Ensure the operating area is clear of obstacles
2. Keep the Perseus-Lite system within sight
3. Be ready to stop operation immediately if needed
4. Monitor the terminal for any error messages

### Initial Testing

Test Perseus-Lite's movement systematically:

1. Begin with slow speeds
2. Test forward and backward motion separately from turning
3. Verify movement matches expected directions
4. Check that the stop command (`k`) works immediately

### Stopping Operation

To stop Perseus-Lite operation:

1. Press `k` in the keyboard control terminal to stop movement
2. Press `Ctrl+C` in the keyboard control terminal to exit
3. Press `Ctrl+C` in the Perseus-Lite system terminal to shutdown

### Troubleshooting

If Perseus-Lite is not responding:

1. Check that the USB connection is secure
2. Verify `/dev/ttyACM0` is accessible
3. Check the Perseus-Lite system terminal for error messages
4. Restart the Perseus-Lite system if necessary

Common issues:

- **No servo response**: Check USB connection and power
- **Permission denied on /dev/ttyACM0**: Add user to dialout group or use sudo
- **Locale warnings**: Use `export LC_ALL=C && export LANG=C` before commands

## System Shutdown

To properly shutdown Perseus-Lite:

1. Stop all movement commands
2. Press `Ctrl+C` in the keyboard control terminal
3. Press `Ctrl+C` in the Perseus-Lite system terminal
4. Disconnect the USB cable
5. Power down Perseus-Lite hardware

# Perseus Lite

Bringup package for the Perseus Lite rover.

## Teleop Operation

### Terminal 1 - Launch the rover

```bash
nix run .#perseus-lite
```

Or with colcon (local development):

```bash
cd ~/perseus-v2/software/ros_ws
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch perseus_lite perseus_lite.launch.py
```

### Terminal 2 - Launch the Xbox controller (wireless)

```bash
nix run .#generic_controller
```

This launches the wireless Xbox controller by default. Other options:

```bash
# Wired Xbox controller
nix run .#generic_controller -- wireless:=false

# Other controllers
nix run .#generic_controller -- type:=8bitdo
nix run .#generic_controller -- type:=logitech wireless:=false
```

### Verify operation

```bash
# Check nodes are running
ROS_DOMAIN_ID=42 ros2 node list

# Expected nodes include:
# /controller_manager
# /diff_drive_base_controller
# /twist_mux
# /joy_node
# /generic_controller

# Check cmd_vel is being published
ROS_DOMAIN_ID=42 ros2 topic echo /cmd_vel
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `False` | Use simulation time |
| `use_mock_hardware` | `False` | Use mock hardware instead of real servos |
| `serial_port` | `/dev/ttyACM0` | ST3215 servo serial port |
| `baud_rate` | `1000000` | Servo baud rate |

## Architecture

```
Controller (xbox/keyboard)
        │
        ▼
    /joy_vel
        │
        ▼
    twist_mux ──── /cmd_vel_nav (from Nav2)
        │
        ▼
    /cmd_vel
        │
        ▼
diff_drive_controller
        │
        ▼
  ST3215 Servos
```

The `twist_mux` node arbitrates between joystick input (`/joy_vel`, priority 10) and navigation commands (`/cmd_vel_nav`, priority 1). Joystick always overrides navigation when active.

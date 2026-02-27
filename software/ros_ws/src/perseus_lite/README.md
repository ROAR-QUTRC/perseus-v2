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

### Terminal 3 - Visualize in rviz2 (on laptop)

```bash
ROS_DOMAIN_ID=42 rviz2
```

On non-NixOS systems, use:

```bash
ROS_DOMAIN_ID=42 nixgl rviz2
```

In rviz2:

1. Set **Fixed Frame** to `odom` or `base_link`
2. Add displays:
   - **Add** → **By topic** → `/scan` → **LaserScan**
   - **Add** → **By topic** → `/tf` → **TF**
   - **Add** → **RobotModel** (uses `/robot_description`)

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

| Argument            | Default        | Description                              |
| ------------------- | -------------- | ---------------------------------------- |
| `use_sim_time`      | `False`        | Use simulation time                      |
| `use_mock_hardware` | `False`        | Use mock hardware instead of real servos |
| `serial_port`       | `/dev/ttyACM0` | ST3215 servo serial port                 |
| `baud_rate`         | `1000000`      | Servo baud rate                          |

## Configuration Files

Perseus Lite uses dedicated config files in `autonomy/config/` with hardware-specific settings:

| Config File                                     | Description                         |
| ----------------------------------------------- | ----------------------------------- |
| `slam_toolbox_params_perseus_lite.yaml`         | SLAM Toolbox (main)                 |
| `slam_toolbox_params_clean_perseus_lite.yaml`   | SLAM Toolbox (tuned for clean maps) |
| `slam_toolbox_params_minimal_perseus_lite.yaml` | SLAM Toolbox (minimal/fast)         |
| `nav_params_perseus_lite.yaml`                  | Nav2 navigation stack               |
| `ekf_params_perseus_lite.yaml`                  | Robot localization EKF              |

These differ from the full Perseus rover configs:

- `base_frame`: `base_link` (Perseus uses `base_footprint`)
- `scan_topic`: `/scan` (Perseus uses `/livox/scan`)
- `robot_radius`: `0.12` (smaller footprint)

Override configs via launch arguments:

```bash
ros2 launch perseus_lite perseus_lite_slam_and_nav2.launch.py \
    slam_params_file:=/path/to/custom_slam.yaml \
    nav_params_file:=/path/to/custom_nav.yaml \
    ekf_params_file:=/path/to/custom_ekf.yaml
```

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

## Teleop with SLAM Mapping

To drive the robot while building a map in real-time:

### Terminal 1 - Launch rover with SLAM and Nav2 (on robot)

```bash
ROS_DOMAIN_ID=42 ros2 launch perseus_lite perseus_lite_slam_and_nav2.launch.py
```

This launches:

- Perseus Lite hardware (diff_drive_controller, LIDAR, IMU)
- SLAM Toolbox for mapping
- Nav2 navigation stack
- EKF for odometry fusion
- twist_mux for cmd_vel arbitration

### Terminal 2 - Launch the Xbox controller (on robot or laptop)

```bash
nix run .#generic_controller
```

**Important**: Hold the **left trigger (LT)** as a dead-man switch while using the left stick to drive. Release LT to stop.

### Terminal 3 - Visualize in rviz2 (on laptop)

```bash
ROS_DOMAIN_ID=42 rviz2
```

On non-NixOS systems:

```bash
ROS_DOMAIN_ID=42 nixgl rviz2
```

In rviz2:

1. Set **Fixed Frame** to `map`
2. Add displays:
   - **Add** → **By topic** → `/map` → **Map** (to see SLAM map)
   - **Add** → **By topic** → `/scan` → **LaserScan**
   - **Add** → **By topic** → `/tf` → **TF**
   - **Add** → **RobotModel** (uses `/robot_description`)

### Verify SLAM is working

```bash
# Check SLAM node is running
ROS_DOMAIN_ID=42 ros2 node list | grep slam

# Expected: /slam_toolbox or /async_slam_toolbox_node

# Check map is being published
ROS_DOMAIN_ID=42 ros2 topic hz /map

# Should show ~0.1-1 Hz when moving
```

### Save the map

```bash
ROS_DOMAIN_ID=42 ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

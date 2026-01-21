# Autonomy Diagnostics Tool

The `autonomy_diagnostics` package provides a real-time TUI (Terminal User Interface) for monitoring the health of the autonomy system. It checks topics, TF frames, Nav2 lifecycle nodes, and configuration files.

## Quick Start

```console
# Run the diagnostics TUI
ros2 run autonomy_diagnostics autonomy_tui

# Or via launch file
ros2 launch autonomy_diagnostics autonomy_diagnostics.launch.py
```

Press `q` to quit.

## What It Monitors

### Topics

| Topic | Expected Rate | Critical | Description |
|-------|---------------|----------|-------------|
| `/scan` | 10 Hz | Yes | 2D LiDAR scans |
| `/imu/data` | 100 Hz | Yes | IMU measurements |
| `/odom` | 50 Hz | Yes | Wheel odometry |
| `/odometry/filtered` | 30 Hz | Yes | EKF-filtered odometry |
| `/map` | 0.1 Hz | No | Occupancy grid map |
| `/cmd_vel` | Variable | No | Velocity commands |
| `/joint_states` | 50 Hz | Yes | Joint feedback |

### TF Frames

The tool verifies the complete transform chain required for navigation:

```
map
 └── odom         (from SLAM/AMCL)
      └── base_link    (from EKF)
           └── chassis      (from URDF)
                ├── laser_2d_frame  (LiDAR mount)
                └── imu_link        (IMU mount)
```

### Nav2 Lifecycle Nodes

Checks the state of all Nav2 lifecycle-managed nodes:

- `bt_navigator`
- `controller_server`
- `planner_server`
- `map_server`
- `amcl`
- `smoother_server`
- `velocity_smoother`
- `collision_monitor`
- `waypoint_follower`
- `behavior_server`

States: `active` (green), `inactive`/`unconfigured` (yellow), `not_found`/`error` (red)

### Configuration Files

Verifies existence of required config files:

- `autonomy/config/ekf_params.yaml`
- `autonomy/config/slam_toolbox_params.yaml`
- `autonomy/config/perseus_nav_params.yaml`
- `autonomy/config/cmd_vel_mux.yaml`

## TUI Layout

```
+==============================================================================+
|            AUTONOMY DIAGNOSTICS - Press 'q' to quit                          |
+==============================================================================+
| SUMMARY: Topics 7/7 | TF 5/5 | Lifecycle 9/11 | Config 4/4                   |
+----------------------------------+-------------------------------------------+
| TOPICS                           | TF FRAMES                                 |
| -------------------------------- | ----------------------------------------- |
| /scan            10.2 Hz   [OK]  | map -> odom           [OK]   SLAM        |
| /imu/data        99.8 Hz   [OK]  | odom -> base_link     [OK]   EKF         |
| /odom            49.5 Hz   [OK]  | base_link -> chassis  [OK]   URDF        |
| /odometry/filtered 30.1 Hz [OK]  | chassis -> laser_2d   [OK]   LiDAR       |
| /map              0.1 Hz   [OK]  | chassis -> imu_link   [OK]   IMU         |
| /cmd_vel          0.0 Hz   [--]  |                                           |
| /joint_states    50.2 Hz   [OK]  |                                           |
+----------------------------------+-------------------------------------------+
| LIFECYCLE NODES                  | CONFIG FILES                              |
| -------------------------------- | ----------------------------------------- |
| bt_navigator        [ACTIVE]     | ekf_params.yaml            [OK]          |
| controller_server   [ACTIVE]     | slam_toolbox_params.yaml   [OK]          |
| planner_server      [ACTIVE]     | perseus_nav_params.yaml    [OK]          |
| map_server          [INACTIVE]   | cmd_vel_mux.yaml           [OK]          |
+----------------------------------+-------------------------------------------+
| Time: 14:32:45 | Refresh: 200ms                                              |
+==============================================================================+
```

## Status Indicators

| Status | Color | Meaning |
|--------|-------|---------|
| `[OK]` | Green | Working correctly |
| `[WARN]` | Yellow | Rate below 80% of expected |
| `[CRIT]` | Red | Rate below 50% of expected |
| `[STALE]` | Red | No messages for > 2 seconds |
| `[NONE]` | Red | No messages received |
| `[FAIL]` | Red | TF transform not available |

## Simple Mode

When running without a TTY (e.g., piped output or non-interactive terminal), the tool automatically falls back to a simple text mode that prints status updates every second.

## Use Cases

### Pre-Flight Check

Run before autonomous operations to verify all systems are operational:

```console
ros2 run autonomy_diagnostics autonomy_tui
```

All critical items should show `[OK]` or `[ACTIVE]` before starting navigation.

### Debugging

When autonomy fails, use the diagnostics tool to quickly identify:

- Missing topics (sensor not running)
- Broken TF chain (localization failed)
- Inactive lifecycle nodes (Nav2 not started)
- Missing config files (package not built)

### Integration Testing

Run alongside the autonomy stack during development to monitor system health in real-time.

## Package Location

```
software/ros_ws/src/autonomy_diagnostics/
├── autonomy_diagnostics/
│   ├── __init__.py
│   └── autonomy_tui.py
├── config/
│   └── diagnostics_config.yaml
├── launch/
│   └── autonomy_diagnostics.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

## Dependencies

- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `nav_msgs`
- `tf2_ros`
- `lifecycle_msgs`
- `curses` (Python standard library)

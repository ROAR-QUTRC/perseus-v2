# crater_exit

ROS2 node that detects when the rover is stuck inside a crater and drives it out by moving forward slowly until the IMU pitch indicates the rim has been crested. After exiting, it automatically re-sends remaining waypoints to Nav2 to resume navigation.

## Problem

When inside a crater, the LiDAR sees crater walls as obstacles in all directions. Nav2 cannot plan a path out because the costmap shows the robot as completely surrounded. The collision monitor also blocks all Nav2-sourced motion since obstacles are within the stop polygon.

## How It Works

1. **Detection**: Monitors the 2D LaserScan for a "crater signature" — a high fraction of rays returning close obstacles in at least 3 of 4 scan quadrants. Requires multiple consecutive confirmations to avoid false positives from walls or corners.

2. **Cancel Nav2 goals**: On detection, cancels the active Nav2 goal via the waypoints bridge (`/autonomy/cancel_waypoints`) to prevent the BT from failing and losing the waypoint list.

3. **Exit**: Publishes a slow forward velocity on a dedicated twist_mux topic at higher priority than Nav2, bypassing the collision monitor. Computes a rolling average of IMU pitch over a configurable time window (default 3s) to filter out vibrations and divots. When the average pitch drops below the threshold (nose consistently pointing down), the rover has crested the crater rim.

4. **Resume navigation**: After exiting, re-sends the remaining waypoints to Nav2 via the waypoints bridge (`/autonomy/run_waypoints`). Navigation continues from where it left off.

### State Machine

```
MONITORING → CRATER_DETECTED → EXITING → CREST_DETECTED → MONITORING
                  |                ↓            |
                  |            (timeout)        |
            cancel Nav2            ↓       re-send waypoints
              goals            MONITORING
```

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Subscribe | 2D laser scan for crater detection |
| `/livox/imu/corrected` | `sensor_msgs/Imu` | Subscribe | IMU for pitch monitoring |
| `/autonomy/active_waypoints` | `geometry_msgs/PoseArray` | Subscribe | Cached waypoints from bridge (transient local) |
| `/autonomy/navigation_info` | `perseus_interfaces/NavigationData` | Subscribe | Nav2 feedback from bridge |
| `/cmd_vel_crater_exit` | `geometry_msgs/TwistStamped` | Publish | Velocity override (twist_mux priority 5) |
| `/crater_exit/status` | `std_msgs/String` | Publish | Current state name |

## Services Called

| Service | Type | Description |
|---------|------|-------------|
| `/autonomy/cancel_waypoints` | `perseus_interfaces/RunWaypoints` | Cancel active Nav2 goals on crater detection |
| `/autonomy/run_waypoints` | `perseus_interfaces/RunWaypoints` | Re-send remaining waypoints after exit |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `scan_topic` | `/scan` | LaserScan topic |
| `imu_topic` | `/livox/imu/corrected` | IMU topic |
| `cmd_vel_out_topic` | `/cmd_vel_crater_exit` | Output velocity topic |
| `obstacle_distance_threshold` | `1.5` m | Rays closer than this count as obstacles |
| `obstacle_coverage_threshold` | `0.7` | Fraction of rays that must be obstacles (0-1) |
| `detection_confirmation_count` | `5` | Consecutive scans confirming crater before acting |
| `min_quadrants_with_obstacles` | `3` | Must see obstacles in at least N of 4 quadrants |
| `exit_linear_speed` | `0.08` m/s | Forward speed during exit |
| `exit_timeout` | `15.0` s | Max exit duration before aborting |
| `pitch_threshold_deg` | `-3.0` deg | Average pitch below this = nose down = crested rim |
| `pitch_window_duration` | `3.0` s | Rolling window for pitch averaging (filters vibration/divots) |
| `enable_crater_exit` | `true` | Master enable/disable |

## Usage

Standalone:
```bash
ros2 launch crater_exit crater_exit.launch.py
```

The node is also included in `autonomy.launch.py`.

## Twist Mux Integration

The node publishes to `/cmd_vel_crater_exit` which is configured in `cmd_vel_mux.yaml` at priority 5:
- Priority 1: Nav2 (`cmd_vel_nav`)
- Priority 5: Crater exit (`/cmd_vel_crater_exit`)
- Priority 10: Teleop (`/diff_base_controller/cmd_vel`)

This means crater exit overrides Nav2 but manual teleop always takes precedence.

## Waypoints Bridge Integration

The `nav2_waypoints_bridge` node publishes active waypoints on `/autonomy/active_waypoints` (PoseArray, transient local QoS) whenever a new goal is sent to Nav2. The crater exit node caches these and uses the `number_of_poses_remaining` from `/autonomy/navigation_info` to determine which waypoints still need to be visited. After exiting a crater, only the remaining (unvisited) waypoints are re-sent.

## Tuning

These parameters will need arena-specific tuning:
- `obstacle_distance_threshold` — depends on crater diameter
- `obstacle_coverage_threshold` — lower values make detection more sensitive
- `pitch_threshold_deg` — depends on crater slope angle and IMU noise
- `pitch_window_duration` — longer window = more robust against vibration but slower to react
- `exit_linear_speed` — slower is safer but takes longer to exit

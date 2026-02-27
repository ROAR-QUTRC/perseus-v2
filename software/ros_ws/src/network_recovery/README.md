# network_recovery

A ROS2 C++ node that monitors WiFi connectivity while the robot operates (teleoperation, waypoint navigation, or autonomous tasks) and autonomously navigates the robot back toward areas of known good signal when connectivity is lost.

The node is designed for robots operating out of sight of the operator for extended periods (30-40+ minutes). It runs passively alongside normal operation and only takes control when a confirmed signal loss occurs.

## What it does

- **WiFi heatmap** — continuously samples robot position + signal strength and publishes a colour-coded `MarkerArray` for RViz (green = strong, yellow = fair, red = poor, black = no signal). The heatmap builds over the entire mission regardless of whether recovery is ever needed.
- **Signal trend detection** — tracks whether signal is stable, improving, or degrading using a sliding-window linear regression over recent RSSI readings.
- **Signal leash** — if signal is consistently degrading while already weak, publishes a warning (and optionally pauses the robot) _before_ connectivity is actually lost. This gives the operator a chance to intervene while comms still work.
- **Breadcrumb trail** — records the robot's path with two-tier storage: regular breadcrumbs (FIFO, up to 500) and pinned breadcrumbs at locations with strong signal (never evicted, up to 50). Spacing adapts based on signal strength — dense near the signal boundary, sparse in good areas.
- **Three-phase recovery** — when connectivity is lost for a configurable timeout:
  1. **Gradual retreat** — moves a short distance (default 3m) back along the breadcrumb trail and checks if signal returns.
  2. **Heatmap-guided navigation** — queries the heatmap for the nearest cell with historically good signal and navigates directly to it via Nav2.
  3. **Full breadcrumb backtrack** — retraces the entire breadcrumb trail in reverse, checking connectivity between batches.
- **Heatmap export** — saves the heatmap to a timestamped JSON file on shutdown for post-mission review.

All recovery navigation uses Nav2's `NavigateThroughPoses` action, which ensures obstacle avoidance remains active during recovery movement.

## State machine

```
MONITORING ──> SIGNAL_DEGRADED ──> SIGNAL_LEASH ──> SIGNAL_LOST_WAITING ──> RECOVERING
     ^                                                                          |
     |                                                                          v
     +──────────────────── RECOVERY_SUCCEEDED <────────────────────────── (signal found)
     |
     +──────────────────── RECOVERY_FAILED ────────────────────────────── (all options exhausted)
```

| State                 | What happens                                                                                           |
| --------------------- | ------------------------------------------------------------------------------------------------------ |
| `MONITORING`          | Normal operation. Records breadcrumbs, builds heatmap. No interference with navigation.                |
| `SIGNAL_DEGRADED`     | RSSI below warning threshold but still connected. Informational only.                                  |
| `SIGNAL_LEASH`        | Signal trend is consistently degrading in a weak area. Publishes warning; optionally pauses the robot. |
| `SIGNAL_LOST_WAITING` | Ping failed. Counts down the timeout (default 30s). Returns to MONITORING if signal comes back.        |
| `RECOVERING`          | Runs three-phase recovery. Checks connectivity between each phase.                                     |
| `RECOVERY_SUCCEEDED`  | Signal reacquired. Holds 3s then returns to MONITORING. Clears regular breadcrumbs, keeps pinned.      |
| `RECOVERY_FAILED`     | All recovery options exhausted. Robot stops. Waits for operator or natural signal return.              |

## Topics

| Topic                                | Type                                       | Direction     | Description                                                                          |
| ------------------------------------ | ------------------------------------------ | ------------- | ------------------------------------------------------------------------------------ |
| `/network_recovery/status`           | `perseus_interfaces/NetworkRecoveryStatus` | Publish       | Current state, RSSI, latency, trend, breadcrumb count, human-readable status message |
| `/network_recovery/heatmap`          | `visualization_msgs/MarkerArray`           | Publish       | Colour-coded grid of signal strength for RViz                                        |
| `/network_recovery/breadcrumb_trail` | `nav_msgs/Path`                            | Publish       | Current breadcrumb trail for RViz                                                    |
| `/navigate_through_poses`            | `nav2_msgs/NavigateThroughPoses`           | Action client | Used during recovery to navigate with obstacle avoidance                             |

All publishers use transient local QoS so late-joining subscribers (e.g. RViz opened after the node starts) receive the latest data.

## Building

```bash
# Build the message dependency first
colcon build --packages-select perseus_interfaces

# Then build the node
colcon build --packages-select network_recovery
```

## Running

Standalone:

```bash
ros2 launch network_recovery network_recovery.launch.py
```

The node is also included automatically in `autonomy.launch.py` when launching the full autonomy stack.

## Configuration

All parameters are in `config/network_recovery_params.yaml`. Key parameters to tune for your environment:

| Parameter                  | Default                    | Description                                                |
| -------------------------- | -------------------------- | ---------------------------------------------------------- |
| `ping_target`              | `""` (auto-detect gateway) | IP to ping for connectivity checks                         |
| `signal_loss_timeout_s`    | `30.0`                     | Seconds of no connectivity before recovery starts          |
| `recovery_enabled`         | `true`                     | Set `false` to disable recovery but keep heatmap and leash |
| `leash_enabled`            | `true`                     | Enable proactive signal degradation warning                |
| `leash_pause_robot`        | `false`                    | If `true`, cancel Nav2 goals when leash triggers           |
| `breadcrumb_max_count`     | `500`                      | Max regular breadcrumbs (sized for 40+ min missions)       |
| `retreat_distance_m`       | `3.0`                      | Phase 1 retreat distance before checking signal            |
| `heatmap_save_on_shutdown` | `true`                     | Save heatmap JSON on node shutdown                         |
| `heatmap_save_directory`   | `~/.ros/network_recovery`  | Where to save heatmap JSON files                           |

Auto-detection: if `ping_target` is empty, the node reads `/proc/net/route` to find the default gateway. If `wifi_interface` is empty, it reads `/proc/net/wireless` to find the active WiFi interface.

## Architecture

```
network_recovery_node (main)
  |
  +-- NetworkMonitor        Daemon thread: ping + RSSI + trend detection
  |
  +-- HeatmapGrid           Grid data structure + MarkerArray + JSON export
  |
  +-- RecoveryNavigator     Nav2 action client + three-phase recovery logic
```

The `NetworkMonitor` runs in its own thread and provides thread-safe accessors for the latest connectivity data. The main node runs timers for pose sampling, breadcrumb recording, status publishing, heatmap publishing, and state machine ticks — all on the ROS executor thread.

## Heatmap export format

On shutdown, the heatmap is saved to `~/.ros/network_recovery/heatmap_YYYYMMDD_HHMMSS.json`:

```json
{
  "map_frame": "map",
  "cell_size_m": 0.5,
  "total_cells": 1234,
  "cells": [
    {"x": 1.25, "y": 2.75, "rssi_avg": -45.2, "rssi_min": -52, "rssi_max": -38, "count": 15},
    ...
  ]
}
```

The heatmap is **not** reloaded on startup because WiFi signal patterns change between runs. The saved files are for post-mission analysis only.

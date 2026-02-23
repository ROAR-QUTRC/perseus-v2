# Autonomy SBC Operation

## Overview

The SBC (Single Board Computer) runs the Livox MID360 LiDAR and the full autonomy stack (SLAM, Nav2, EKF). To prevent high-bandwidth pointcloud data (~20-50 MB/s) from saturating the WiFi network, DDS is restricted to localhost on the SBC. A separate topic republisher bridges only low-bandwidth control topics to/from the network.

```
SBC (localhost-only DDS, domain 42)
+-----------------------------------------------+
|  Livox MID360 -> /livox/lidar (stays local!)   |
|  SLAM Toolbox <- /livox/lidar, /livox/imu      |
|  Nav2 Stack   <- /map, /scan, /odom            |
|  EKF          <- /livox/imu, /odom             |
|                                                |
|  foxglove_bridge (WebSocket :8765)             |
|    -> Selective topics via TCP (not DDS)       |
+-----------------------------------------------+
         | lo interface (DDS)
+-----------------------------------------------+
|  Topic Republisher (lo + network DDS)          |
|    Bridges: /cmd_vel, /tf, /map, /odom, etc.   |
+-----------------------------------------------+
         | network interface (DDS)
+-----------------------------------------------+
|  Main Rover PC (network DDS, domain 42)        |
|  RSP + Controllers + twist_mux -> CAN bus      |
+-----------------------------------------------+
```

**Key insight:** The main SBC stack and the republisher use different CycloneDDS configs but the same domain ID (42). The main stack only sees localhost; the republisher sees both localhost and network, forwarding selected topics between them.

## Setup

### 1. Run the setup script

```console
nix run .#autonomy-sbc-setup
```

This creates:
- `~/.config/cyclonedds/localhost.xml` — DDS restricted to loopback
- `~/.config/cyclonedds/bridge.xml` — DDS on loopback + network
- `~/.config/cyclonedds/env.sh` — exports `CYCLONEDDS_URI` for the localhost config

### 2. Source the environment

Add to your shell profile (`~/.bashrc` or `~/.zshrc`):

```bash
source ~/.config/cyclonedds/env.sh
```

Then restart your shell or run `source ~/.config/cyclonedds/env.sh`.

## Running

From within the dev shell:

```console
nix run .#perseus-autonomy-sbc
```

This command:
1. Checks that `CYCLONEDDS_URI` is set and points to the localhost config
2. Launches the main SBC stack (SLAM + Nav2 + EKF + Livox + foxglove_bridge)
3. After a 5-second delay, launches the topic republisher with the bridge CycloneDDS config
4. Handles SIGINT/SIGTERM to cleanly shut down both processes

## Foxglove Connection

Connect Foxglove Studio to `ws://<sbc-ip>:8765` to visualize topics. This uses WebSocket (TCP), not DDS, so pointcloud data can be streamed without affecting the rover's WiFi network.

## Monitoring

Run the monitor script within the dev shell:

```console
nix run .#ros2 -- run autonomy autonomy-sbc-monitor.sh
```

Or directly (if `ros2` is on PATH):

```bash
autonomy-sbc-monitor
```

This shows:
- CycloneDDS configuration status
- Active ROS topics
- Pointcloud topic bandwidth (should be localhost-only)
- Bridge topic status

## Bridged Topics

**Outbound (SBC -> network):** `/cmd_vel`, `/tf`, `/tf_static`, `/map`, `/odom`, `/scan`, `/robot_description`, `/joint_states`, `/plan`, Nav2 action topics

**Inbound (network -> SBC):** `/goal_pose`, `/cmd_vel_joy`

**Excluded:** `/livox/lidar` (pointcloud) — too high bandwidth for WiFi

The full list is in `software/ros_ws/src/autonomy/config/sbc_bridge_topics.yaml`.

## Reversing Configuration

To remove all SBC-specific CycloneDDS configuration:

```console
nix run .#autonomy-sbc-setup -- --reverse
```

Then remove the `source ~/.config/cyclonedds/env.sh` line from your shell profile.

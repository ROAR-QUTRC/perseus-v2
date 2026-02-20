# mapping_autotune

Automated SLAM parameter tuning for Perseus Lite. Drives the robot through repeatable maneuvers, captures the resulting occupancy grid, scores map quality, and stores everything in SQLite for review.

## Prerequisites

- Perseus Lite running with SLAM and Nav2:
  ```bash
  ros2 launch perseus_lite perseus_lite_slam_and_nav2.launch.py
  ```
- Robot on flat ground, stationary, with LIDAR and odometry active.

## Quick Start

### 1. Set up the database

```bash
# On the robot (creates /opt/mapping_autotune/autotune.db)
ros2 run mapping_autotune setup_mapping_db.sh

# Or specify a custom directory
./scripts/setup_mapping_db.sh --db-dir ~/autotune_data
```

### 2. Run the autotune

```bash
# Quick test (1 run)
ros2 run mapping_autotune autotune_node --ros-args -p max_runs:=1

# Full session (10 runs across all phases)
ros2 run mapping_autotune autotune_node

# Named session with more runs
ros2 run mapping_autotune autotune_node --ros-args \
  -p max_runs:=20 \
  -p session_name:="living_room_test"
```

Or use the launch file (also starts the IMU filter node):

```bash
ros2 launch mapping_autotune autotune.launch.py max_runs:=15
```

### 3. Review results

```bash
# Interactive TUI
ros2 run mapping_autotune review_tui --ros-args \
  -p db_path:=/opt/mapping_autotune/autotune.db

# Or with CLI arg
ros2 run mapping_autotune review_tui -- --db-path /opt/mapping_autotune/autotune.db
```

### 4. Export best parameters

From the review TUI, press `e` on the runs screen to export the best-performing parameters as a YAML file ready to drop into `autonomy/config/`.

To export a markdown report:

```bash
ros2 run mapping_autotune export_report -- \
  --db-path /opt/mapping_autotune/autotune.db \
  --session-id 1
```

## How It Works

### State Machine

```
PREFLIGHT -> CONFIGURING -> RESETTING_SLAM -> SETTLING -> RUNNING_MANEUVER -> CAPTURING_MAP -> ANALYZING
                 ^                                                                                |
                 +------- next run ----------------------------------------------------------------+
                                                                                                  |
                                                                                             COMPLETE
```

The node runs a pre-flight check (database, SLAM, EKF, LIDAR, odometry, IMU), then iterates through parameter combinations. For each run it:

1. Writes modified SLAM/EKF config to temp YAML files
2. Restarts SLAM (and EKF if needed) with the new parameters
3. Waits for SLAM to settle
4. Executes a maneuver (drives a pattern while logging odometry and IMU)
5. Captures the occupancy grid map
6. Analyzes map quality with 6 metrics
7. Stores everything in the database

### Tuning Phases

Each phase uses the best result from the previous phase as its baseline:

| Phase | Category | Runs | Parameters |
|-------|----------|------|------------|
| 0 | IMU Calibration | 0 | Gyro bias and deadband (stationary, 30s) |
| 1 | IMU Integration | 3 | IMU off / on / on+deadband in EKF |
| 2 | SLAM Rotation | 10 | `minimum_travel_heading`, `correlation_search_space_smear_deviation`, `angle_variance_penalty` |
| 3 | Scan Matching | 6 | `minimum_angle_penalty`, `link_match_minimum_response_fine`, `minimum_distance_penalty` |
| 4 | Timing | 4 | `throttle_scans`, `minimum_time_interval`, `transform_publish_period` |
| 5 | Speed | 3 | Rotation and linear speed during maneuver |
| 6 | EKF Noise | 3 | Process noise covariance for yaw and angular velocity |

When `max_runs` is less than the full plan, the system prioritizes phases 1-3 (highest impact on rotation smearing).

### Map Quality Metrics

All scores are 0.0-1.0 (higher is better):

| Metric | Weight | What it measures |
|--------|--------|------------------|
| Wall Straightness | 25% | Deviation of occupied cells from Hough-detected lines |
| Wall Thickness | 20% | How close wall thickness is to the ideal of 1 cell |
| Ghost Walls | 20% | Parallel line pairs indicating doubled/smeared walls |
| Symmetry | 15% | Structural symmetry via PCA and mirrored IoU |
| Free Space | 10% | Spurious isolated occupied cells in free areas |
| Occupied Density | 10% | Whether occupied cell ratio falls in the expected 2-10% range |

## Autotune Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `db_path` | `/opt/mapping_autotune/autotune.db` | SQLite database path |
| `max_runs` | `10` | Total runs across all phases |
| `session_name` | auto-generated | Name for this session |
| `session_description` | `""` | Optional description |
| `settling_time` | `3.0` | Seconds to wait after SLAM restart |
| `maneuver_pattern` | `box_return` | `box_return` or `corridor` |
| `slam_config_path` | auto-detected | Baseline SLAM YAML |
| `ekf_config_path` | auto-detected | Baseline EKF YAML |
| `skip_preflight` | `false` | Skip pre-flight checks |

## Review TUI

The interactive terminal UI has four screens:

- **Sessions** -- list all autotune sessions with run counts and best scores
- **Runs** -- table of all runs in a session with per-metric scores and ratings
- **Detail** -- full parameter diff, metric bar charts, and ASCII map preview
- **Compare** -- side-by-side comparison of two runs

Key bindings:

| Key | Action |
|-----|--------|
| `Enter` | Open selected item |
| `Esc` | Go back |
| `1`-`5` | Rate the selected run |
| `n` | Add notes to a run |
| `c` | Compare two runs (press on first, then second) |
| `e` | Export best parameters as YAML |
| `r` | Export session report (markdown) |
| `q` | Quit |

## Remote Server Setup

Store the database on a separate machine for team review:

```bash
# On the server
./scripts/setup_mapping_db.sh --server --db-dir /srv/mapping_autotune

# On the robot
./scripts/setup_mapping_db.sh --remote-sync \
  --ssh-host 192.168.1.100 \
  --ssh-user mapper \
  --ssh-key ~/.ssh/id_ed25519 \
  --remote-db-dir /srv/mapping_autotune

# Review from the server (no ROS needed)
python3 scripts/review_tui_standalone.py --db-path /srv/mapping_autotune/autotune.db
```

The robot syncs the database to the server after each run via rsync over SSH.

## Running Tests

```bash
colcon build --packages-select mapping_autotune
source install/setup.bash
python3 -m pytest src/mapping_autotune/test/ -v
```

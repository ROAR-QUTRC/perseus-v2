# Perseus Lite Drifting

Plays a drift sound effect through the USB speaker when Perseus Lite is drifting — simultaneous forward motion and rotation.

## How It Works

The node subscribes to `/odometry/filtered` (EKF output) and detects drifting when the robot has both significant linear velocity and angular velocity at the same time. When triggered, it plays `drifting.mp3` via `mpg123` and enforces a configurable cooldown to prevent the sound repeating too frequently.

## Prerequisites

```bash
# ALSA utilities (should already be installed from voice control setup)
sudo apt-get install alsa-utils

# MP3 command-line player
sudo apt-get install mpg123
```

The USB speaker should already be configured per the voice control package setup (`feat/pereus-lite-voice-control`).

## Building

```bash
colcon build --packages-select perseus_lite_drifting
source install/setup.bash
```

## Usage

### Launch with config

```bash
ros2 launch perseus_lite_drifting drifting.launch.py
```

### Run standalone

```bash
ros2 run perseus_lite_drifting drift_detector_node
```

### Override parameters at launch

```bash
ros2 run perseus_lite_drifting drift_detector_node --ros-args \
  -p cooldown_seconds:=15.0 \
  -p min_angular_velocity:=0.5
```

## Configuration

All parameters are in `config/drifting_params.yaml`:

| Parameter              | Default  | Description                                    |
| ---------------------- | -------- | ---------------------------------------------- |
| `min_linear_velocity`  | `0.1`    | Minimum forward speed (m/s) to count as moving |
| `min_angular_velocity` | `0.3`    | Minimum yaw rate (rad/s) to count as rotating  |
| `cooldown_seconds`     | `30.0`   | Minimum seconds between audio plays            |
| `audio_player`         | `mpg123` | Command-line MP3 player binary                 |

## Topics

| Topic                | Type                | Direction  | Description                |
| -------------------- | ------------------- | ---------- | -------------------------- |
| `/odometry/filtered` | `nav_msgs/Odometry` | Subscribed | Filtered odometry from EKF |

## Package Structure

```
perseus_lite_drifting/
├── audio/
│   └── drifting.mp3               # Sound effect
├── config/
│   └── drifting_params.yaml       # Tunable parameters
├── launch/
│   └── drifting.launch.py         # Launch file with config
├── perseus_lite_drifting/
│   ├── __init__.py
│   └── drift_detector_node.py     # Main node
├── resource/
│   └── perseus_lite_drifting      # ament index marker
├── package.xml
├── setup.cfg
└── setup.py
```

# AS7343 Spectral Sensor Driver

ROS 2 driver for the **ams-OSRAM AS7343** 14-channel multi-spectral sensor over I2C, designed for Raspberry Pi 4 and Raspberry Pi 5.

## Overview

The AS7343 measures light intensity across 14 spectral channels spanning 405 nm (violet) to 855 nm (near-infrared), plus broadband visible and flicker detection channels. This package provides a ROS 2 node that reads the sensor via I2C and publishes the spectral data as ROS topics.

## Spectral Channels

| Channel | Wavelength | FWHM | Region |
|---------|-----------|------|--------|
| F1 | 405 nm | ~30 nm | Violet |
| F2 | 425 nm | ~22 nm | Violet-Blue |
| FZ | 450 nm | ~55 nm | Blue (wide) |
| F3 | 475 nm | ~30 nm | Blue-Cyan |
| F4 | 515 nm | ~40 nm | Green-Cyan |
| F5 | 550 nm | ~35 nm | Green |
| FY | 555 nm | ~100 nm | Yellow-Green (wide) |
| FXL | 600 nm | ~80 nm | Orange (wide) |
| F6 | 640 nm | ~50 nm | Red |
| F7 | 690 nm | ~55 nm | Deep Red |
| F8 | 745 nm | ~50 nm | Near-IR |
| NIR | 855 nm | ~20 nm | Near-IR |
| VIS | Broadband | Full visible | Clear channel |
| FD | N/A | N/A | Flicker detection |

## Hardware

- **Sensor:** AS7343 (I2C address `0x39`)
- **Breakout boards:** SparkFun AS7343 Qwiic, Adafruit AS7343, Pimoroni AS7343
- **Host:** Raspberry Pi 4 / Raspberry Pi 5
- **Wiring:** VCC→3.3V, GND→GND, SDA→GPIO2, SCL→GPIO3

Verify sensor detection:
```bash
sudo apt-get install -y i2c-tools
i2cdetect -y 1
# Should show 0x39
```

## Build

```bash
cd ~/perseus-v2/software/ros_ws
colcon build --packages-select perseus_interfaces as7343_driver
source install/setup.bash
```

## Run

```bash
# Using launch file (loads default config)
ros2 launch as7343_driver as7343.launch.py

# With overrides
ros2 launch as7343_driver as7343.launch.py i2c_bus:=/dev/i2c-1 required:=true

# Direct run
ros2 run as7343_driver as7343_node --ros-args -p gain:=128 -p publish_rate_hz:=10.0
```

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~/spectral_data` | `perseus_interfaces/SpectralData` | 14-channel spectral readings + status |
| `~/flicker_status` | `perseus_interfaces/FlickerStatus` | Ambient light flicker detection (50/60 Hz) |
| `~/integration_time_ms` | `std_msgs/Float64` | Current integration time in ms |

## Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `i2c_bus` | `/dev/i2c-1` | I2C bus path |
| `device_address` | `0x39` | I2C address |
| `gain` | `256` | ADC gain (1–2048) |
| `atime` | `29` | Integration time multiplier (0–255) |
| `astep` | `599` | Integration step (0–65535) |
| `smux_mode` | `18` | Channels: 6, 12, or 18 |
| `publish_rate_hz` | `5.0` | Publish rate in Hz |
| `led_enabled` | `false` | Enable on-board LED |
| `flicker_detection_enabled` | `true` | Enable flicker detection |

Integration time: `(atime + 1) × (astep + 1) × 2.78 µs` — default ~50 ms per cycle, ~150 ms for 18-channel mode.

## Documentation

Full Sphinx documentation is in `docs/`. Build with:
```bash
cd docs && sphinx-build -b html . _build/html
```

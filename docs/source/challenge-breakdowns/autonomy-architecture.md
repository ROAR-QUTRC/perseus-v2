# System Architecture

## Data Flow

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Sensors   │───▶│  Perception │───▶│ Localisation│───▶│    Nav2     │
│ LiDAR, IMU  │    │ SLAM Toolbox│    │  EKF, AMCL  │    │  Planner    │
└─────────────┘    └─────────────┘    └─────────────┘    └──────┬──────┘
                                                                │
┌─────────────┐    ┌─────────────┐    ┌─────────────┐           │
│  Actuators  │◀───│ Controllers │◀───│ Cmd Vel Mux │◀──────────┘
│   Wheels    │    │ DiffDrive   │    │  Priority   │
└─────────────┘    └─────────────┘    └─────────────┘
```

## TF Tree Structure

```
map (world-fixed, from SLAM/AMCL)
  │
  └── odom (local odometry frame, from EKF)
        │
        └── base_link (robot centre)
              ├── base_footprint
              ├── imu_link
              └── chassis
                    ├── camera_link ─── camera_link_optical
                    ├── laser_2d_frame (2D LiDAR)
                    ├── livox_frame (3D LiDAR)
                    ├── flange_bearing ─── differential_bar
                    ├── left_rocker
                    │     ├── front_left_motor ─── front_left_wheel
                    │     └── rear_left_motor ─── rear_left_wheel
                    └── right_rocker
                          ├── front_right_motor ─── front_right_wheel
                          └── rear_right_motor ─── rear_right_wheel
```

## Coordinate Conventions (REP-103)

| Frame | +X | +Y | +Z |
|-------|----|----|-----|
| `base_link` | Forward | Left | Up |
| `imu_link` | Forward | Left | Up |
| `camera_link_optical` | Right | Down | Forward (optical axis) |
| Wheel joints | — | Rotation axis | — |

For complete frame definitions, see [Robot Description](../development/software/robot-description.md).

## Component Responsibilities

### Robot State Publisher
- Publishes static TF transforms from URDF
- Provides `base_link` → sensor frame transforms

### DiffDrive Controller
- Converts `/cmd_vel` to wheel velocities
- Publishes wheel odometry on `/odom`
- Manages joint state broadcasting

### EKF (robot_localization)
- Fuses wheel odometry + IMU data
- Publishes filtered odometry on `/odometry/filtered`
- Provides `odom` → `base_link` transform

### SLAM Toolbox
- Builds occupancy grid from laser scans
- Publishes `map` → `odom` transform
- Handles loop closure and map optimisation

### Nav2 Stack
- Global path planning (NavFn A*)
- Local trajectory planning (DWB)
- Behaviour tree execution
- Recovery behaviours

### Twist Mux
- Arbitrates between velocity command sources
- Priority-based topic selection
- Enables safe manual override

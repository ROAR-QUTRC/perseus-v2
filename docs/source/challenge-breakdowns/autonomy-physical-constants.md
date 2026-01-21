# Physical Constants and Geometry

:::{warning}
Changing these values requires **recalibration, retuning, and validation** of the entire autonomy stack.
:::

## Drivetrain Geometry

| Parameter | Value | Unit | Config Location |
|-----------|-------|------|-----------------|
| Wheel separation (track width) | **0.714** | m | `perseus_controllers.yaml` |
| Wheel radius | **0.147** | m | `perseus_controllers.yaml` |
| Wheel diameter | 0.30 | m | URDF |
| Wheel width | 0.14 | m | URDF |
| Wheels per side | 2 | — | `perseus_controllers.yaml` |

### Why These Matter

- **Wheel separation** directly affects angular velocity calculations. Wrong value = robot turns incorrectly.
- **Wheel radius** affects linear velocity calculations. Wrong value = robot travels wrong distance.
- Both values are used by the DiffDriveController for odometry computation.

## Chassis Dimensions

| Parameter | Value | Unit |
|-----------|-------|------|
| Chassis length | 0.6 | m |
| Chassis width | 0.45 | m |
| Chassis height | 0.2 | m |
| Robot radius (footprint) | **0.12** | m |

### Impact on Navigation

The robot radius is used by:
- Costmap inflation layer
- Collision checking
- Path planning clearance

## Kinematic Limits

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Max linear velocity | 1.7 | m/s | Hardware limit |
| Max angular velocity | 5.0 | rad/s | Hardware limit |
| Max linear acceleration | 10.0 | m/s² | Hardware limit |
| Max angular acceleration | 10.0 | rad/s² | Hardware limit |

### Nav2 Conservative Limits

| Parameter | Value | Unit | Rationale |
|-----------|-------|------|-----------|
| Nav2 max linear velocity | **0.2** | m/s | Safe for indoor/competition |
| Nav2 max angular velocity | **0.8** | rad/s | Prevents aggressive rotation |
| Nav2 linear acceleration | 2.0 | m/s² | Smooth motion |
| Nav2 angular acceleration | 2.5 | rad/s² | Smooth rotation |

## Sensor Specifications

### 2D LiDAR (M2M2)

| Parameter | Value |
|-----------|-------|
| Update rate | 10 Hz |
| Horizontal samples | 720 |
| Field of view | 360° |
| Range | 0.05 – 200.0 m |
| Resolution | 0.01 m |
| Default IP | 192.168.1.137 |
| Default port | 1446 |

### IMU (LSM6DSOX)

| Parameter | Value |
|-----------|-------|
| I2C bus | `/dev/i2c-7` |
| I2C address | 0x6A |
| Update rate | 100 Hz |
| Frame ID | `imu_link` |

### 3D LiDAR (Livox MID-360)

| Parameter | Value |
|-----------|-------|
| Update rate | 50 Hz |
| Vertical FoV | -7° to +52° |
| Max range | 30.0 m |
| Resolution | 0.013 m |

## Mass Properties

| Component | Mass (kg) |
|-----------|-----------|
| Chassis | 2.77 |
| Each wheel | 1.0 |
| Each motor | 5.24 |
| Rocker arm | 0.5 |
| Differential bar | 0.92 |

These values are used in Gazebo simulation for realistic physics.

# Interface Contracts

:::{warning}
Changes to items in this section **will break autonomy**. Coordinate with the autonomy team before modifying any contract item.
:::

## TF Frames (Must Exist)

| Frame | Publisher | Notes |
|-------|-----------|-------|
| `map` → `odom` | AMCL or SLAM Toolbox | Global localisation correction |
| `odom` → `base_link` | EKF (`robot_localization`) | Local odometry estimate |
| `base_link` → `chassis` | Robot State Publisher | Fixed transform from URDF |
| `chassis` → `laser_2d_frame` | Robot State Publisher | LiDAR mounting position |
| `chassis` → `imu_link` | Robot State Publisher | IMU mounting position |

### Frame Naming Requirements

| Sensor | Required Frame Name |
|--------|---------------------|
| 2D LiDAR | `laser_2d_frame` |
| 3D LiDAR | `livox_frame` |
| IMU | `imu_link` |
| Camera optical | `camera_link_optical` |

## Topics and Message Types (Must Match)

| Topic | Message Type | Publisher | Frequency | QoS |
|-------|-------------|-----------|-----------|-----|
| `/cmd_vel` | `geometry_msgs/Twist` | Twist Mux output | Variable | Default |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | Nav2 Controller | 10 Hz | Default |
| `/odom` | `nav_msgs/Odometry` | DiffDriveController | 50 Hz | Default |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF | 30 Hz | Default |
| `/scan` | `sensor_msgs/LaserScan` | M2M2 LiDAR driver | 10 Hz | Best Effort |
| `/imu/data` | `sensor_msgs/Imu` | IMU driver | 100 Hz | Default |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox | 0.1 Hz | Transient Local |
| `/tf` | `tf2_msgs/TFMessage` | Multiple | Various | Default |
| `/tf_static` | `tf2_msgs/TFMessage` | RSP | Once | Transient Local |

## ros2_control Hardware Interface (Must Match)

### Controller Manager

- Update rate: **50 Hz**
- Lock memory: `false` (not real-time system)

### Wheel Joint Names (exact strings)

```yaml
left_wheel_names:
  - front_left_wheel_joint
  - rear_left_wheel_joint
right_wheel_names:
  - front_right_wheel_joint
  - rear_right_wheel_joint
```

:::{danger}
Changing joint names requires updating the URDF, controller config, and any nodes that reference joints directly.
:::

### State Interfaces

| Interface | Unit | Description |
|-----------|------|-------------|
| `position` | rad | Wheel angular position |
| `velocity` | rad/s | Wheel angular velocity |
| `current` | A | Motor current (mapped from `effort`) |
| `temperature` | °C | Motor temperature |

### Command Interface

| Interface | Unit | Description |
|-----------|------|-------------|
| `velocity` | rad/s | Commanded wheel velocity |

### Supported Hardware Plugins

| Plugin | Use Case |
|--------|----------|
| `perseus_hardware/VescSystemHardware` | Real robot (VESC controllers) |
| `perseus_hardware/McbSystemHardware` | Real robot (MCB) |
| `gz_ros2_control/GazeboSimSystem` | Gazebo simulation |
| `mock_components/GenericSystem` | Testing without hardware |

### Configuration File

`software/ros_ws/src/perseus/config/perseus_controllers.yaml`

## Velocity Command Multiplexing

The twist mux arbitrates between command sources by priority:

| Source | Topic | Priority | Timeout |
|--------|-------|----------|---------|
| Navigation | `/cmd_vel_nav` | 1 (highest) | 0.5 s |
| Direct control | `/diff_base_controller/cmd_vel` | 10 | 0.2 s |

Output: `/cmd_vel` → Controller Manager

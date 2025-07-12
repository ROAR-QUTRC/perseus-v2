# I2C IMU Driver Implementation Plan

## Overview

This document outlines the professional implementation plan for adding an optional I2C IMU sensor to the Perseus v2 rover system. The IMU will be directly connected to I2C bus 7 on the host system at address 0x6A.

## Project Context

Based on analysis of the Perseus v2 codebase, this implementation follows established patterns for:

- Optional sensor integration (M2M2 LiDAR pattern)
- Hardware interface abstraction (ST3215 servo system pattern)
- Linux device access (file descriptor management)
- ROS 2 sensor integration (sensor_msgs/Imu publishing)

## Hardware Specifications

- **I2C Bus**: `/dev/i2c-7`
- **Device Address**: `0x6A`
- **Connection**: Direct host I2C interface (no ESP32 firmware layer)
- **Availability**: Optional hardware - system must function without it

## Package Structure

```
perseus_sensors/i2c_imu_driver/
├── CMakeLists.txt
├── package.xml
├── include/i2c_imu_driver/
│   ├── i2c_imu_node.hpp
│   ├── i2c_imu_hardware_interface.hpp
│   └── i2c_device.hpp
├── src/
│   ├── i2c_imu_node.cpp
│   ├── i2c_imu_hardware_interface.cpp
│   ├── i2c_device.cpp
│   └── main.cpp
├── launch/
│   └── i2c_imu.launch.py
├── config/
│   └── i2c_imu_config.yaml
└── urdf/
    └── i2c_imu.urdf.xacro
```

## Implementation Architecture

### 1. Hardware Interface Layer

**Class: `I2cImuHardwareInterface`**

- Extends `hardware_interface::SensorInterface`
- Manages I2C device lifecycle (configure → activate → read → deactivate)
- Provides sensor state interfaces for ROS 2 Control

**Key Features:**

- Device detection with graceful degradation
- Async I/O with dedicated read thread
- Timeout protection for I2C operations
- Error handling with retry mechanisms

### 2. I2C Device Abstraction

**Class: `I2cDevice`**

- Low-level I2C communication wrapper
- Uses Linux I2C API (`/dev/i2c-*`)
- Implements FdWrapper pattern for resource management
- Provides register read/write methods

**Key Features:**

- Device file descriptor management
- I2C transaction handling
- Error checking and timeout protection
- Register abstraction layer

### 3. ROS 2 Node Implementation

**Class: `I2cImuNode`**

- Standalone ROS 2 node for IMU data publishing
- Publishes `sensor_msgs/Imu` messages
- Timer-based periodic reading (following M2M2 LiDAR pattern)
- Parameter-based configuration

**Key Features:**

- Configurable update rate
- Frame ID management
- Covariance matrix handling
- Graceful failure handling

## Configuration System

### Launch Configuration

```python
# i2c_imu.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'i2c_bus',
            default_value='/dev/i2c-7',
            description='I2C bus device file'
        ),
        DeclareLaunchArgument(
            'device_address',
            default_value='0x6A',
            description='I2C device address'
        ),
        DeclareLaunchArgument(
            'update_rate',
            default_value='100',
            description='IMU update rate in Hz'
        ),
        Node(
            package='i2c_imu_driver',
            executable='i2c_imu_node',
            name='i2c_imu_node',
            parameters=[{
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'device_address': LaunchConfiguration('device_address'),
                'update_rate': LaunchConfiguration('update_rate'),
                'frame_id': 'imu_link'
            }],
            output='screen'
        )
    ])
```

### Parameter Configuration

```yaml
# i2c_imu_config.yaml
i2c_imu:
  i2c_bus: "/dev/i2c-7"
  device_address: 0x6A
  update_rate: 100
  frame_id: "imu_link"
  required: false # Optional hardware
  timeout_ms: 1000
  retry_count: 3

  # IMU-specific parameters
  accel_range: 2 # ±2g
  gyro_range: 250 # ±250°/s
  filter_bandwidth: 42 # Hz

  # Covariance matrices (if known)
  orientation_covariance: [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
  angular_velocity_covariance: [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
  linear_acceleration_covariance: [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
```

### URDF Integration

```xml
<!-- i2c_imu.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="i2c_imu" params="parent_link xyz rpy">

    <joint name="imu_joint" type="fixed">
      <parent link="${parent_link}"/>
      <child link="imu_link"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
    </joint>

    <link name="imu_link">
      <visual>
        <geometry>
          <box size="0.02 0.02 0.005"/>
        </geometry>
        <material name="imu_material">
          <color rgba="0.0 0.0 1.0 1.0"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <box size="0.02 0.02 0.005"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.001"/>
        <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
      </inertial>
    </link>

    <gazebo reference="imu_link">
      <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
        <frame_name>imu_link</frame_name>
        <ros>
          <namespace>/</namespace>
          <remapping>~/out:=imu/data</remapping>
        </ros>
      </plugin>
    </gazebo>

  </xacro:macro>

</robot>
```

## Integration with Main System

### Main Launch File Integration

```python
# In perseus.launch.py or perseus_lite.launch.py
DeclareLaunchArgument(
    'use_i2c_imu',
    default_value='true',
    description='Enable I2C IMU sensor'
),

# Conditional inclusion
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('i2c_imu_driver'),
            'launch',
            'i2c_imu.launch.py'
        ])
    ]),
    condition=IfCondition(LaunchConfiguration('use_i2c_imu'))
)
```

### URDF Integration

```xml
<!-- In main robot URDF -->
<xacro:include filename="$(find i2c_imu_driver)/urdf/i2c_imu.urdf.xacro"/>

<!-- Add IMU to robot -->
<xacro:i2c_imu parent_link="base_link" xyz="0.0 0.0 0.1" rpy="0 0 0"/>
```

## Implementation Details

### 1. Device Detection and Initialization

```cpp
// Follows BQ769x2 pattern from firmware
class I2cDevice {
public:
    I2cDevice(const std::string& bus, uint8_t address);
    bool initialize();
    bool isConnected();

private:
    FdWrapper i2c_fd_;
    uint8_t device_address_;
    bool performDeviceDetection();
};
```

### 2. Hardware Interface Implementation

```cpp
// Follows ST3215 hardware interface pattern
class I2cImuHardwareInterface : public hardware_interface::SensorInterface {
public:
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    std::unique_ptr<I2cDevice> i2c_device_;
    std::thread read_thread_;
    std::atomic<bool> reading_active_{false};
};
```

### 3. Error Handling Strategy

**Graceful Degradation:**

- IMU absence doesn't prevent system startup
- Warning logs but continues operation
- Navigation stack adapts to available sensors

**Robust Communication:**

- Device detection with retry mechanisms
- Timeout protection for I2C operations
- Meaningful error messages and logging
- Connection monitoring during operation

### 4. Data Processing Pipeline

```cpp
// IMU data structure
struct ImuData {
    float accel_x, accel_y, accel_z;      // m/s²
    float gyro_x, gyro_y, gyro_z;         // rad/s
    float temp;                           // °C
    std::chrono::steady_clock::time_point timestamp;
};

// Processing pipeline
ImuData raw_data = readRawData();
ImuData calibrated_data = applyCalibration(raw_data);
sensor_msgs::msg::Imu imu_msg = convertToRosMessage(calibrated_data);
```

## Dependencies

### Package Dependencies

```xml
<!-- package.xml -->
<depend>rclcpp</depend>
<depend>rclcpp_lifecycle</depend>
<depend>hardware_interface</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>tf2</depend>
<depend>tf2_geometry_msgs</depend>

<!-- Build dependencies -->
<build_depend>fd-wrapper</build_depend>  <!-- Shared library -->
```

### System Dependencies

- Linux I2C development headers (`libi2c-dev`)
- Appropriate I2C device permissions
- User in `i2c` group or appropriate udev rules

## Testing Strategy

### Unit Testing

```cpp
// Mock I2C interface for testing
class MockI2cDevice : public I2cDevice {
    // Override methods for testing
};

// Test scenarios:
// - Device detection success/failure
// - Communication timeout handling
// - Data parsing and validation
// - Parameter configuration
```

### Integration Testing

- With and without physical IMU present
- Launch file parameter variations
- System startup with optional hardware
- ROS 2 Control integration testing

### Hardware Testing

- I2C bus communication verification
- Device address detection
- Data accuracy validation
- Long-term reliability testing

## Deployment Considerations

### System Configuration

```bash
# Ensure I2C bus is available
ls /dev/i2c-*

# Check device detection
i2cdetect -y 7

# Verify device at address 0x6A
i2cget -y 7 0x6A 0x00
```

### Permissions Setup

```bash
# Add user to i2c group
sudo usermod -a -G i2c $USER

# Or create udev rule
echo 'SUBSYSTEM=="i2c-dev", MODE="0666"' | sudo tee /etc/udev/rules.d/99-i2c.rules
sudo udevadm control --reload-rules
```

## Future Enhancements

1. **Multi-IMU Support**: Extend to support multiple IMU sensors
2. **Calibration Tools**: Add calibration utilities for bias correction
3. **Magnetometer Support**: Extend for 9-DOF IMU sensors
4. **Sensor Fusion**: Integration with robot localization package
5. **Hardware Interface Plugin**: ros2_control integration for standardized access

## Conclusion

This implementation plan follows Perseus v2's established patterns for professional sensor integration while maintaining system robustness and configurability. The design ensures the I2C IMU is truly optional and won't affect system operation if absent, while providing high-quality sensor data when available.

The implementation leverages existing infrastructure (FdWrapper, hardware interface patterns, sensor integration patterns) to maintain consistency with the broader codebase architecture.

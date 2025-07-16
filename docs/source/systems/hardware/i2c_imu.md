# I2C IMU

The Perseus V2 system includes an I2C IMU (Inertial Measurement Unit) driver that provides orientation, angular velocity, and linear acceleration data through ROS2.

## Supported IMU Types

The driver supports multiple IMU devices:

- **LSM6DSOX** (0x6A) - 6-axis accelerometer + gyroscope (default)
- **MPU6050** (0x68) - 6-axis accelerometer + gyroscope
- **MPU9250** (0x68) - 9-axis accelerometer + gyroscope + magnetometer
- **LSM6DS3** (0x6A) - 6-axis accelerometer + gyroscope

## Building and Running

### Enter Development Environment

```bash
nix develop
```

### Build the Package

```bash
colcon build --packages-select perseus_sensors
```

### Source the Environment

```bash
source install/setup.bash
```

### Launch the IMU Node

```bash
ros2 launch perseus_sensors i2c_imu.launch.py
```

## Configuration

### Using Different IMU Types

To use a different IMU type, modify the config file at `src/perseus_sensors/i2c_imu_driver/config/i2c_imu_config.yaml`

Example configuration for MPU6050:

```yaml
i2c_imu:
  ros__parameters:
    device_type: "mpu6050"
    i2c_bus: "/dev/i2c-7"
    device_address: 104
    update_rate: 100.0
    frame_id: "imu_link"
```

### Multiple IMUs

For multiple IMUs of the same type, use different I2C addresses:

```yaml
# First IMU
i2c_imu_1:
  ros__parameters:
    device_type: "lsm6dsox"
    device_address: 106
    frame_id: "imu_1_link"

# Second IMU
i2c_imu_2:
  ros__parameters:
    device_type: "lsm6dsox"
    device_address: 107
    frame_id: "imu_2_link"
```

## Troubleshooting

### Common Issues

1. **Permission denied on I2C bus**: Ensure the user has access to the I2C device
2. **Device not found**: Check if the IMU is properly connected and the correct address is used
3. **Wrong device type**: Verify the device_type parameter matches your physical IMU

### Verification

To verify the IMU is working correctly:

```bash
ros2 topic echo /imu/data
```

The node publishes sensor_msgs/Imu messages containing orientation, angular velocity, and linear acceleration data.

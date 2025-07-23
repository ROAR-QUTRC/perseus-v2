from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "i2c_bus", default_value="/dev/i2c-7", description="I2C bus device file"
            ),
            DeclareLaunchArgument(
                "device_address",
                default_value="106",
                description="I2C device address (0x6A = 106 decimal)",
            ),
            DeclareLaunchArgument(
                "update_rate",
                default_value="100.0",
                description="IMU update rate in Hz",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="imu_link",
                description="Frame ID for IMU data",
            ),
            DeclareLaunchArgument(
                "required",
                default_value="false",
                description="Whether IMU is required for system operation",
            ),
            DeclareLaunchArgument(
                "timeout_ms",
                default_value="1000",
                description="I2C operation timeout in milliseconds",
            ),
            DeclareLaunchArgument(
                "retry_count",
                default_value="3",
                description="Number of retries for I2C operations",
            ),
            DeclareLaunchArgument(
                "use_config_file",
                default_value="false",
                description="Whether to use configuration file",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value="i2c_imu_config.yaml",
                description="Configuration file name",
            ),
            # I2C IMU node
            Node(
                package="perseus_sensors",
                executable="i2c_imu_node",
                name="i2c_imu_node",
                parameters=[
                    {
                        "i2c_bus": LaunchConfiguration("i2c_bus"),
                        "device_address": LaunchConfiguration("device_address"),
                        "update_rate": LaunchConfiguration("update_rate"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "required": LaunchConfiguration("required"),
                        "timeout_ms": LaunchConfiguration("timeout_ms"),
                        "retry_count": LaunchConfiguration("retry_count"),
                        # Default calibration parameters
                        "accel_scale_x": 1.0,
                        "accel_scale_y": 1.0,
                        "accel_scale_z": 1.0,
                        "accel_offset_x": 0.0,
                        "accel_offset_y": 0.0,
                        "accel_offset_z": 0.0,
                        "gyro_scale_x": 1.0,
                        "gyro_scale_y": 1.0,
                        "gyro_scale_z": 1.0,
                        "gyro_offset_x": 0.0,
                        "gyro_offset_y": 0.0,
                        "gyro_offset_z": 0.0,
                        # Default covariance matrices
                        "orientation_covariance": [
                            0.1,
                            0.0,
                            0.0,
                            0.0,
                            0.1,
                            0.0,
                            0.0,
                            0.0,
                            0.1,
                        ],
                        "angular_velocity_covariance": [
                            0.01,
                            0.0,
                            0.0,
                            0.0,
                            0.01,
                            0.0,
                            0.0,
                            0.0,
                            0.01,
                        ],
                        "linear_acceleration_covariance": [
                            0.1,
                            0.0,
                            0.0,
                            0.0,
                            0.1,
                            0.0,
                            0.0,
                            0.0,
                            0.1,
                        ],
                    }
                ],
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=2.0,
            ),
        ]
    )

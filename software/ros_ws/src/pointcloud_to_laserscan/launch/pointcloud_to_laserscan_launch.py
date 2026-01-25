from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="scanner",
                default_value="scanner",
                description="converts lidar pointcloud to scan",
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                remappings=[
                    ("cloud_in", "/livox/lidar"),
                    ("scan", "/lidar_scan"),
                ],
                parameters=[
                    {
                        "target_frame": "livox_frame",
                        "transform_tolerance": 0.1,1
                        "min_height": -2.0,
                        "max_height": 0.2,# keep points below 0.1m to meet lidar specs
                        "range_min": 1.0,
                        "angle_increment": 0.0174533,
                        "scan_time": 0.1,
                        "range_max": 40.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
                name="pointcloud_to_laserscan",
            ),
        ]
    )

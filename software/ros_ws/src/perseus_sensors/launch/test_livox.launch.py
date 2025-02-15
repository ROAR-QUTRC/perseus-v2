from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_lidar_publisher",
                parameters=[
                    {
                        "xfer_format": 0,
                        "multi_topic": 1,
                        "data_src": 0,
                        "publish_freq": 10.0,
                        "output_data_type": 0,
                        "frame_id": "livox_frame",
                        "user_config_path": "$(find perseus_sensors)/config/livox_config.json",
                    }
                ],
                output="screen",
            ),
            Node(package="rviz2", executable="rviz2", name="rviz2"),
        ]
    )

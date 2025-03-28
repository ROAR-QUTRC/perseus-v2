from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Construct the path to the config file
    config_path = PathJoinSubstitution(
        [FindPackageShare("perseus_sensors"), "config", "livox_config.json"]
    )

    return LaunchDescription(
        [
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_lidar_publisher",
                parameters=[
                    {
                        "xfer_format": 0,
                        "multi_topic": 0,
                        "data_src": 0,
                        "publish_freq": 10.0,
                        "output_data_type": 0,
                        "frame_id": "livox_frame",
                        "user_config_path": config_path,  # Use the resolved path here
                    }
                ],
                output="screen",
            ),
        ]
    )

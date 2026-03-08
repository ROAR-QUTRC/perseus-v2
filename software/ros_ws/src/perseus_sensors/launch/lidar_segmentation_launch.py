from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    # Correct path to YAML config
    config_file = os.path.join(
        get_package_share_directory('perseus_sensors'),
        'config',
        'lidar_segmentation.yaml'
    )

    lidar_node = Node(
        package='perseus_sensors',
        executable='lidar_segmentation_node',
        name='lidar_segmentation_node',  # <-- must match YAML
        output='screen',
        parameters=[config_file]
    )

    rviz_file = os.path.join(
        get_package_share_directory('perseus_sensors'),
        'rviz',
        'lidar_segmentation_view.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_file],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='screen'
    )

    return LaunchDescription([
        launch_rviz_arg,
        lidar_node,
        rviz_node
    ])

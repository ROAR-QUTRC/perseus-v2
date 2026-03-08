from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    filter_node = Node(
        package='perseus_mapping',
        executable='sim_pointcloud_filter',
        name='sim_pointcloud_filter',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([filter_node])
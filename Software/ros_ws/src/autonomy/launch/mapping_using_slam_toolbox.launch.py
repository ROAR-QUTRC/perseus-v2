from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# Assumptions
# SLAM parameters will be stored in autonomy/config/slam_params.yaml
# RViz configuration will be stored in autonomy/rviz/perseus_slam.rviz

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # Declare launch arguments
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    
    # Assuming we'll store SLAM parameters in the autonomy package
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("autonomy"),
                                 'config', 'slam_params.yaml'),
        description='Full path to the ROS2 parameters file for SLAM Toolbox'
    )

    # SLAM Toolbox node
    slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    # RViz node
    # Assuming we'll create a custom RViz config in the autonomy package
    rviz_config_dir = os.path.join(
        get_package_share_directory('autonomy'),
        'rviz',
        'perseus_slam.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    # Create and return launch description
    ld = LaunchDescription()
    
    # Add all actions to launch description
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)

    return ld
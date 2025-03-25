from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file')
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('autonomy'),
            'config',
            'cmd_vel_mux.yaml'
        ]),
        description='Full path to the ROS2 parameters file for cmd_vel multiplexer'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    mux_node = Node(
        package='topic_tools',
        executable='mux',
        name='cmd_vel_mux',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            params_file
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        mux_node
    ])
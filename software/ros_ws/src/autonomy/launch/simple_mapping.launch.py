from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    slam_params_file = LaunchConfiguration("slam_params_file")

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'perseus/base_link/laser', 'laser_frame']
    )
    
    declare_slam_params_file = DeclareLaunchArgument(
        "slam_params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("autonomy"),
            "config",
            "mapper_params_online_async_v2.yaml"
        ]),
        description="Full path to the ROS2 parameters file for slam_toolbox"
    )

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_file]
    )

    ld = LaunchDescription()
    ld.add_action(declare_slam_params_file)
    ld.add_action(slam_toolbox_node)
    return ld
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    ekf_config_file = LaunchConfiguration("ekf_config")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_ekf_config = DeclareLaunchArgument(
        "ekf_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("autonomy"), "config", "ekf_config_stereo_odom.yaml"]
        ),
        description="Full path to the ROS2 parameters file for EKF",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([declare_use_sim_time, declare_ekf_config, ekf_node])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    ekf_config_file = LaunchConfiguration("ekf_config")
    imu_topic = LaunchConfiguration("imu_topic", default="/livox/imu")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_ekf_config = DeclareLaunchArgument(
        "ekf_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("autonomy"), "config", "ekf_config.yaml"]
        ),
        description="Full path to the ROS2 parameters file for EKF",
    )

    declare_imu_topic = DeclareLaunchArgument(
        "imu_topic",
        default_value="/livox/imu",
        description="IMU topic name",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config_file,
            {"use_sim_time": use_sim_time, "imu0": imu_topic},
        ],
    )

    return LaunchDescription(
        [declare_use_sim_time, declare_ekf_config, declare_imu_topic, ekf_node]
    )

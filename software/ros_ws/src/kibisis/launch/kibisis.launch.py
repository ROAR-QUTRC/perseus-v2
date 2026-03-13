from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    hardware_plugin = LaunchConfiguration("hardware_plugin")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use simulation time",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="False",
            description="Use mock hardware (mirrors commands to states)",
        ),
        DeclareLaunchArgument(
            "hardware_plugin",
            default_value="kibisis_hardware/KibisisSystemHardware",
            choices=[
                "mock_components/GenericSystem",
                "gz_ros2_control/GazeboSimSystem",
                "kibisis_hardware/KibisisSystemHardware",
            ],
            description="ros2_control hardware plugin to use",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel",
            description="Topic for velocity commands",
        ),
    ]

    def robot_state_publisher(context):
        performed_use_mock = IfCondition(use_mock_hardware).evaluate(context)
        final_plugin = (
            "mock_components/GenericSystem"
            if performed_use_mock
            else hardware_plugin
        )
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("kibisis"),
                        "launch",
                        "robot_state_publisher.launch.py",
                    ])
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "hardware_plugin": final_plugin,
                }.items(),
            )
        ]

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("kibisis"),
                "launch",
                "controllers.launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "cmd_vel_topic": cmd_vel_topic,
        }.items(),
    )

    return LaunchDescription(arguments + [
        OpaqueFunction(function=robot_state_publisher),
        controllers_launch,
    ])

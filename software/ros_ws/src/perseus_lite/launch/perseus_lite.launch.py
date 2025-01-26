from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ARGUMENTS
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    hardware_plugin = LaunchConfiguration("hardware_plugin")
    # can_bus = LaunchConfiguration("can_bus")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use time provided by simulation",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="False",
            description="Use mock hardware components which mirror commands to state interfaces (overrides hardware_plugin)",
        ),
        DeclareLaunchArgument(
            "hardware_plugin",
            default_value="perseus_lite_hardware/ST3215SystemHardware",
            choices=[
                "mock_components/GenericSystem",
                # "perseus_hardware/VescSystemHardware",
                # "perseus_hardware/McbSystemHardware",
                "gz_ros2_control/GazeboSimSystem",
                "perseus_lite_hardware/ST3215SystemHardware",
            ],
            description="The hardware plugin to use for ros2_control",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Serial port for ST3215 servos",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="1000000",
            description="Baud rate for ST3215 servos",
        ),
        # DeclareLaunchArgument(
        #    "can_bus",
        #    default_value="can0",
        #    description="CAN bus to use for hardware communications",
        # ),
    ]

    # IMPORTED LAUNCH FILES
    def robot_state_publisher(context):
        performed_use_mock_hardware = IfCondition(use_mock_hardware).evaluate(context)
        final_hardware_plugin = (
            "mock_components/GenericSystem"
            if performed_use_mock_hardware
            else hardware_plugin
        )
        rsp_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("perseus_lite"),
                            "launch",
                            "robot_state_publisher.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "hardware_plugin": final_hardware_plugin,
                # "can_bus": can_bus,
                "serial_port": serial_port,
                "baud_rate": baud_rate,
            }.items(),
        )
        return [rsp_launch]

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus_lite"),
                        "launch",
                        "controllers.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )
    launch_files = [
        OpaqueFunction(function=robot_state_publisher),
        controllers_launch,
    ]

    return LaunchDescription(arguments + launch_files)

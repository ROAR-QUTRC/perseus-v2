
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    # Include the main servo simulation launch file
    servo_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("perseus_payloads"),
                    "launch",
                    "servo_sim.launch.py",
                ]
            )
        ),
        launch_arguments={"use_mock_hardware": use_mock_hardware}.items(),
    )
    # Include the arm/driver launch file (ONLY if NOT using mock hardware)
    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("perseus_payloads"),
                    "launch",
                    "arm.launch.py",
                ]
            )
        ),
        condition=UnlessCondition(use_mock_hardware),
    )
    # Launch RQT GUI for visualization
    rqt_node = Node(
        package="rqt_gui",
        executable="rqt_gui",
        name="rqt_gui",
        output="screen",
    )
    return LaunchDescription(declared_arguments + [servo_sim_launch, arm_launch, rqt_node])

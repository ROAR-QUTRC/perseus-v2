from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ARGUMENTS
    use_sim_time = LaunchConfiguration("use_sim_time", default=False)
    launch_controller_manager = LaunchConfiguration(
        "launch_controller_manager", default="true"
    )
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic", default="/cmd_vel")
    use_sim_time_param = {"use_sim_time": use_sim_time}

    arguments = []

    # CONFIG + DATA FILES
    controller_config = PathJoinSubstitution(
        [FindPackageShare("perseus_lite"), "config", "perseus_lite_controllers.yaml"]
    )

    # NODES
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config, use_sim_time_param],
        output="both",  # output to both screen and log file
        remappings=[],
        condition=IfCondition(launch_controller_manager),
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[use_sim_time_param],
    )
    base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_base_controller",
            "--controller-manager",
            "controller_manager",
            "--controller-ros-args",
            ["--remap /diff_drive_base_controller/cmd_vel:=", cmd_vel_topic],
        ],
        output="screen",
        parameters=[use_sim_time_param],
    )

    nodes = [
        controller_manager,
        base_controller_spawner,
        joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(arguments + nodes)

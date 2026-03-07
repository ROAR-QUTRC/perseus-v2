from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
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
        [FindPackageShare("kibisis"), "config", "kibisis_controllers.yaml"]
    )

    # NODES
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config, use_sim_time_param],
        output="both",
        remappings=[
            ("/robot_description", "/kibisis/robot_description"),
        ],
        condition=IfCondition(launch_controller_manager),
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "30",
        ],
        parameters=[use_sim_time_param],
    )
    base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_base_controller",
            "--controller-manager",
            "controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
        parameters=[use_sim_time_param],
        remappings=[
            ("/diff_drive_base_controller/cmd_vel", ["/kibisis",cmd_vel_topic]),
            ("/robot_description", "/kibisis/robot_description"),
        ],
    )

    # Launch controller_manager and joint_state_broadcaster first
    nodes = [
        controller_manager,
        joint_state_broadcaster_spawner,
    ]

    # EVENT HANDLERS - launch base_controller after joint_state_broadcaster completes
    handlers = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[base_controller_spawner],
            )
        ),
    ]

    return LaunchDescription(arguments + nodes + handlers)

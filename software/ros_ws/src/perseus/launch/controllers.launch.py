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
    use_sim_time_param = {"use_sim_time": use_sim_time}

    arguments = []

    # CONFIG + DATA FILES
    controller_config = PathJoinSubstitution(
        [FindPackageShare("perseus"), "config", "perseus_controllers.yaml"]
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
        arguments=[
            "joint_state_broadcaster",
        ],
        parameters=[use_sim_time_param],
    )
    base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_base_controller",
            "--controller-ros-args",
            "--remap /diff_drive_base_controller/cmd_vel:=/cmd_vel_out --remap /diff_drive_base_controller/odom:=/odom",
        ],
        parameters=[use_sim_time_param],
    )

    # NOTE: There was a comment in one of the ROS2 Control examples
    # about launching the controllers *after* the controller manager
    # to help with "flaky tests" (ie, using RegisterEventHandler with OnProcessExit)
    # to launch them in sequence
    nodes = [
        controller_manager,
        joint_state_broadcaster_spawner,
        # base_controller_spawner,
    ]

    # EVENT HANDLERS
    handlers = [
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         # target_action=gz_spawn_entity,  # after gz spawn or after CM launch
        #         target_action=controller_manager,  # after gz spawn or after CM launch
        #         on_exit=[joint_state_broadcaster_spawner],
        #     )
        # ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[base_controller_spawner],
            )
        ),
    ]
    return LaunchDescription(arguments + nodes + handlers)

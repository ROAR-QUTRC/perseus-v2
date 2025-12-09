from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ARGUMENTS
    gz_world = LaunchConfiguration("gz_world", default="perseus_arc_world.world")

    # CONFIG + DATA FILES
    gz_bridge_params = PathJoinSubstitution(
        [FindPackageShare("perseus_simulation"), "config", "gz_bridge.yaml"]
    )
    gz_world_path = PathJoinSubstitution(
        [FindPackageShare("perseus_simulation"), "worlds", gz_world]
    )

    arguments = [
        DeclareLaunchArgument(
            "gz_world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("perseus_simulation"), "worlds", gz_world]
            ),
            description="The world file from `perseus_simulation` to use",
        ),
        DeclareLaunchArgument(
            "initial_pose_x",
            default_value="-3.5",
            description="Initial X position of the robot",
        ),
        DeclareLaunchArgument(
            "initial_pose_y",
            default_value="-3.0",
            description="Initial Y position of the robot",
        ),
        DeclareLaunchArgument(
            "initial_pose_z",
            default_value="0.3",
            description="Initial Z position of the robot",
        ),
        DeclareLaunchArgument(
            "initial_pose_yaw",
            default_value="0.0",
            description="Initial yaw of the robot",
        ),
    ]

    # IMPORTED LAUNCH FILES
    def gz_launch(context):
        # Perform the world path substitution
        performed_gz_world_path = gz_world_path.perform(context)

        model_path = os.path.join(
            get_package_share_directory("perseus_simulation"), "models"
        )

        # 🔧 Important change: call nixGL directly, just like in your working shell command
        gz_launch = ExecuteProcess(
            cmd=[
                "nixGL",
                "ros2",
                "launch",
                "ros_gz_sim",
                "gz_sim.launch.py",
                f"gz_args:=-r -v 4 {performed_gz_world_path}",
            ],
            output="both",
            additional_env={
                "QT_QPA_PLATFORM": "xcb",
                "QT_SCREEN_SCALE_FACTORS": "1",
                "PROJ_IGNORE_CELESTIAL_BODY": "YES",
                "GZ_SIM_RESOURCE_PATH": model_path,
            },
        )

        return [gz_launch]

    launch_files = [
        OpaqueFunction(function=gz_launch),
    ]

    # NODES
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": gz_bridge_params}],
        output="both",
    )

    # Spawn entity with initial pose parameters
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "perseus",
            "-allow_renaming",
            "true",
            "-x",
            LaunchConfiguration("initial_pose_x"),
            "-y",
            LaunchConfiguration("initial_pose_y"),
            "-z",
            LaunchConfiguration("initial_pose_z"),
            "-Y",
            LaunchConfiguration("initial_pose_yaw"),
        ],
        output="both",
    )

    nodes = [
        gz_bridge,
        gz_spawn_entity,
    ]

    return LaunchDescription(arguments + launch_files + nodes)
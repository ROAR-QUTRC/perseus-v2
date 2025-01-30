from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ARGUMENTS
    gz_world = LaunchConfiguration("gz_world", default="perseus_world.sdf")

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
    ]

    # IMPORTED LAUNCH FILES
    def gz_launch(context):
        # normally this would be handled by including the launch description,
        # but it needs to be wrapped with nixGL, which makes things difficult.
        # As such, we need to just run it directly - but we also need to resolve the world substitution,
        # hence wrapping in an OpaqueFunction.
        performed_gz_world_path = gz_world_path.perform(context)
        gz_launch = ExecuteProcess(
            cmd=[
                "nix",
                "run",
                "--impure",
                "github:nix-community/nixGL",
                "--",
                "ros2",
                "launch",
                "ros_gz_sim",
                "gz_sim.launch.py",
                f"gz_args:=-r -v 4 {performed_gz_world_path}",
            ],
            output="both",
            additional_env={
                "NIXPKGS_ALLOW_UNFREE": "1",
                "QT_QPA_PLATFORM": "xcb",
                "QT_SCREEN_SCALE_FACTORS": "1",
            },
        )

        return [gz_launch]

    # gz_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [
    #             PathJoinSubstitution(
    #                 [
    #                     FindPackageShare("ros_gz_sim"),
    #                     "launch",
    #                     "gz_sim.launch.py",
    #                 ]
    #             )
    #         ]
    #     ),
    #     launch_arguments=[("gz_args", [" -r -v 4 ", gz_world_path])],
    # )
    launch_files = [
        # gz_launch,
        OpaqueFunction(function=gz_launch),
    ]

    # NODES
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": gz_bridge_params}],
        output="both",
    )
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
            "-z",
            "0.5",
        ],
        output="both",
    )

    nodes = [
        gz_bridge,
        gz_spawn_entity,
    ]

    return LaunchDescription(arguments + launch_files + nodes)

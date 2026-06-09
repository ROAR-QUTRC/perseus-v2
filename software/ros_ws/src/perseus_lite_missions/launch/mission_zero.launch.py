"""
Mission Zero — Selene Base Rescue launch.

Composes the perseus_lite simulation, slam_toolbox + Nav2 with deliberately
broken parameters, and the mission orchestrator + checker nodes. The
intentional faults (the wrong values in the two YAML files) are the heart of
the script; the viewer fixes them mid-mission.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    mission_yaml = LaunchConfiguration("mission_yaml")

    declare_args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock",
        ),
        DeclareLaunchArgument(
            "gz_world",
            default_value="selene_base.sdf",
            description="Gazebo world file (overrides perseus_sim default)",
        ),
        DeclareLaunchArgument(
            "initial_pose_x",
            default_value="0.0",
            description="Rover spawn X (open area west of the habitat)",
        ),
        DeclareLaunchArgument(
            "initial_pose_y",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "initial_pose_z",
            default_value="0.3",
        ),
        DeclareLaunchArgument(
            "initial_pose_yaw",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("perseus_lite_missions"),
                    "missions",
                    "m00-cold-open",
                    "configs",
                    "slam_params.broken.yaml",
                ]
            ),
            description="SLAM toolbox params (defaults to the broken Challenge 1 starting state)",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("perseus_lite_missions"),
                    "missions",
                    "m00-cold-open",
                    "configs",
                    "nav2_params.broken.yaml",
                ]
            ),
            description="Nav2 params (defaults to the broken Challenge 2 starting state)",
        ),
        DeclareLaunchArgument(
            "mission_yaml",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("perseus_lite_missions"),
                    "missions",
                    "m00-cold-open",
                    "mission.yaml",
                ]
            ),
            description="Mission descriptor consumed by the orchestrator",
        ),
    ]

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus_lite_simulation"),
                        "launch",
                        "perseus_sim.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "gz_world": LaunchConfiguration("gz_world"),
            "initial_pose_x": LaunchConfiguration("initial_pose_x"),
            "initial_pose_y": LaunchConfiguration("initial_pose_y"),
            "initial_pose_z": LaunchConfiguration("initial_pose_z"),
            "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
        }.items(),
    )

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
    )

    # Static TF for the map<->odom edge while slam_toolbox isn't yet
    # publishing a real one. The orchestrator can remove this once mapping
    # is producing transforms.
    static_map_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_odom_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("autonomy"),
                        "launch",
                        "perseus_nav_bringup.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
            "use_composition": "False",
        }.items(),
    )
    # Delay Nav2 until Gazebo, controllers, and SLAM have had time to come
    # up. Mirrors the controllers_launch delay in perseus_sim.launch.py.
    nav2_bringup_delayed = TimerAction(
        period=12.0,
        actions=[nav2_bringup_launch],
    )

    orchestrator = Node(
        package="perseus_lite_missions",
        executable="orchestrator_node",
        name="mission_orchestrator",
        output="screen",
        parameters=[
            {
                "mission_yaml": mission_yaml,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    map_quality_checker = Node(
        package="perseus_lite_missions",
        executable="map_quality_checker_node",
        name="map_quality_checker",
        output="screen",
        parameters=[
            {
                "slam_params_file": slam_params_file,
                "min_resolution_inverse": 10.0,  # resolution <= 0.1 m
                "min_max_laser_range": 8.0,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    goal_reached_checker = Node(
        package="perseus_lite_missions",
        executable="goal_reached_checker_node",
        name="goal_reached_checker",
        output="screen",
        parameters=[
            {
                "mission_yaml": mission_yaml,
                "tolerance_m": 0.5,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    return LaunchDescription(
        declare_args
        + [
            sim_launch,
            static_map_odom_tf,
            slam_toolbox_node,
            nav2_bringup_delayed,
            orchestrator,
            map_quality_checker,
            goal_reached_checker,
        ]
    )

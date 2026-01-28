import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LoadComposableNodes, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory("autonomy")

    # ARGUMENTS
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    hardware_plugin = LaunchConfiguration("hardware_plugin")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    # SLAM arguments
    slam_params_file = LaunchConfiguration("slam_params_file")

    # Robot localization arguments
    ekf_params_file = LaunchConfiguration("ekf_params_file")

    # Nav2 arguments
    namespace = LaunchConfiguration("namespace")
    autostart = LaunchConfiguration("autostart")
    nav_params_file = LaunchConfiguration("nav_params_file")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    # Nav2 lifecycle nodes (no map_server or amcl needed for SLAM mode - SLAM Toolbox provides the map)
    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "collision_monitor",
        "bt_navigator",
        "waypoint_follower",
        "docking_server",
    ]

    arguments = [
        # Perseus Lite arguments
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
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel",
            description="Topic name for cmd_vel commands (mux output)",
        ),
        # SLAM arguments
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("autonomy"),
                    "config",
                    "slam_toolbox_params_perseus_lite.yaml",
                ]
            ),
            description="Full path to the ROS2 parameters file for SLAM Toolbox",
        ),
        # Robot localization arguments
        DeclareLaunchArgument(
            "ekf_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("autonomy"), "config", "ekf_params_perseus_lite.yaml"]
            ),
            description="Full path to the ROS2 parameters file for robot_localization EKF",
        ),
        # Nav2 arguments
        DeclareLaunchArgument(
            "namespace", default_value="", description="Top-level namespace"
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically startup the nav2 stack",
        ),
        DeclareLaunchArgument(
            "nav_params_file",
            default_value=os.path.join(
                bringup_dir, "config", "nav_params_perseus_lite.yaml"
            ),
            description="Full path to the ROS2 parameters file to use for all launched nodes",
        ),
        DeclareLaunchArgument(
            "use_composition",
            default_value="False",
            description="Use composed bringup if True",
        ),
        DeclareLaunchArgument(
            "container_name",
            default_value="nav2_container",
            description="the name of container that nodes will load in if use composition",
        ),
        DeclareLaunchArgument(
            "use_respawn",
            default_value="False",
            description="Whether to respawn if a node crashes. Applied when composition is disabled.",
        ),
        DeclareLaunchArgument(
            "log_level", default_value="info", description="log level"
        ),
    ]

    # Include perseus_lite launch file
    perseus_lite_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus_lite"),
                        "launch",
                        "perseus_lite.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_mock_hardware": use_mock_hardware,
            "hardware_plugin": hardware_plugin,
            "serial_port": serial_port,
            "baud_rate": baud_rate,
            "cmd_vel_topic": cmd_vel_topic,
        }.items(),
    )

    # Robot localization EKF node
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params_file, {"use_sim_time": use_sim_time}],
        remappings=[
            ("odometry/filtered", "odometry/filtered"),
        ],
    )

    # SLAM Toolbox (use_cuda disabled in slam_toolbox_params.yaml to avoid Jetson CUDA crash)
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
            "autostart": "true",
            "use_lifecycle_manager": "false",
        }.items(),
    )

    # Nav2 remappings
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Create parameter substitutions for nav2
    param_substitutions = {"autostart": autostart}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav_params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Environment variable for logging
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    # Nav2 nodes (non-composable)
    # Note: map_server and amcl are NOT included here because SLAM Toolbox provides the map
    load_nav2_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            SetParameter("use_sim_time", use_sim_time),
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="opennav_docking",
                executable="opennav_docking",
                name="docking_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[{"autostart": autostart}, {"node_names": lifecycle_nodes}],
            ),
        ],
    )

    # Nav2 composable nodes
    # Note: map_server and amcl are NOT included here because SLAM Toolbox provides the map
    load_composable_nav2_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter("use_sim_time", use_sim_time),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package="nav2_controller",
                        plugin="nav2_controller::ControllerServer",
                        name="controller_server",
                        parameters=[configured_params],
                        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                    ),
                    ComposableNode(
                        package="nav2_smoother",
                        plugin="nav2_smoother::SmootherServer",
                        name="smoother_server",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_planner",
                        plugin="nav2_planner::PlannerServer",
                        name="planner_server",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_behaviors",
                        plugin="behavior_server::BehaviorServer",
                        name="behavior_server",
                        parameters=[configured_params],
                        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                    ),
                    ComposableNode(
                        package="nav2_bt_navigator",
                        plugin="nav2_bt_navigator::BtNavigator",
                        name="bt_navigator",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_waypoint_follower",
                        plugin="nav2_waypoint_follower::WaypointFollower",
                        name="waypoint_follower",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_velocity_smoother",
                        plugin="nav2_velocity_smoother::VelocitySmoother",
                        name="velocity_smoother",
                        parameters=[configured_params],
                        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                    ),
                    ComposableNode(
                        package="nav2_collision_monitor",
                        plugin="nav2_collision_monitor::CollisionMonitor",
                        name="collision_monitor",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="opennav_docking",
                        plugin="opennav_docking::DockingServer",
                        name="docking_server",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_lifecycle_manager",
                        plugin="nav2_lifecycle_manager::LifecycleManager",
                        name="lifecycle_manager_navigation",
                        parameters=[
                            {"autostart": autostart, "node_names": lifecycle_nodes}
                        ],
                    ),
                ],
            ),
        ],
    )

    # NOTE: twist_mux is already launched by perseus_lite.launch.py
    # Do not launch cmd_vel_mux.launch.py here to avoid duplicates

    launch_files = [
        stdout_linebuf_envvar,
        perseus_lite_launch,
        ekf_node,
        slam_toolbox,
        load_nav2_nodes,
        load_composable_nav2_nodes,
    ]

    return LaunchDescription(arguments + launch_files)

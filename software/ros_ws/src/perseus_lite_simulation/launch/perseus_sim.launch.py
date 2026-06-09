from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)


def generate_launch_description():
    # ARGUMENTS
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_ekf = LaunchConfiguration("launch_ekf")
    gz_world = LaunchConfiguration("gz_world")

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock",
        ),
        DeclareLaunchArgument(
            "launch_ekf",
            default_value="false",
            description="If true, launch the EKF filter node",
        ),
        DeclareLaunchArgument(
            "gz_world",
            default_value="perseus_arc_world.world",
            description="The world file from `perseus_lite_simulation` to use",
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
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus_lite_simulation"),
                        "launch",
                        "gazebo.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "gz_world": gz_world,
            "initial_pose_x": LaunchConfiguration("initial_pose_x"),
            "initial_pose_y": LaunchConfiguration("initial_pose_y"),
            "initial_pose_z": LaunchConfiguration("initial_pose_z"),
            "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
        }.items(),
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
            "hardware_plugin": "gz_ros2_control/GazeboSimSystem",
        }.items(),
    )
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
            "launch_controller_manager": "false",
        }.items(),
    )
    # Delay controller startup until Gazebo and ros2_control have had time to
    # spawn the robot and expose the control interfaces. The 10 s value is a
    # conservative fallback for slower first runs or heavier worlds; if startup
    # sequencing changes, this is the place to tune or replace with an event-
    # driven trigger.
    controllers_launch_delayed = TimerAction(
        period=10.0,
        actions=[controllers_launch],
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("perseus_lite_simulation"), "rviz", "view.rviz"]
    )
    ekf_config_file = PathJoinSubstitution(
        [FindPackageShare("perseus_lite_simulation"), "config", "ekf_sim_config.yaml"]
    )
    rviz = ExecuteProcess(
        cmd=[
            "nix",
            "run",
            "--impure",
            "github:nix-community/nixGL",
            "--",
            "rviz2",
            "-d",
            rviz_config,
        ],
        output="screen",
        additional_env={
            "NIXPKGS_ALLOW_UNFREE": "1",
            "QT_QPA_PLATFORM": "xcb",
            "QT_SCREEN_SCALE_FACTORS": "1",
            "ROS_NAMESPACE": "/",
            "RMW_QOS_POLICY_HISTORY": "keep_last",
            "RMW_QOS_POLICY_DEPTH": "100",
        },
    )

    # EKF node - only run if launch_ekf parameter is true
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_file, {"use_sim_time": use_sim_time}],
        # Explicit remapping to ensure proper topic connections
        remappings=[
            ("/odometry/filtered", "/odometry/filtered"),  # EKF output
        ],
    )
    # Add delay to EKF to ensure all other nodes are ready
    ekf_delayed = TimerAction(
        period=5.0,
        actions=[ekf_node],
        condition=IfCondition(launch_ekf),
    )
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("rosbridge_server"),
                        "launch",
                        "rosbridge_websocket_launch.xml",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus_lite_simulation"),
                        "launch",
                        "twist_mux.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )
    launch_files = [
        gz_launch,
        rsp_launch,
        controllers_launch_delayed,
        ekf_delayed,
        rosbridge_launch,
        twist_mux_launch,
        rviz,
    ]

    return LaunchDescription(arguments + launch_files)

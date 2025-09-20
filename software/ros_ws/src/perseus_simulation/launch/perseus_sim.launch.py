from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # ARGUMENTS
    use_sim_time = LaunchConfiguration("use_sim_time")

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock",
        ),
    ]
    # RViz configuration file
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("perseus_simulation"), "rviz", "view.rviz"]
    )

    ekf_config_file = PathJoinSubstitution(
        [FindPackageShare("perseus_simulation"), "config", "ekf_config.yaml"]
    )
    # IMPORTED LAUNCH FILES
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus_simulation"),
                        "launch",
                        "gazebo.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus"),
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
                        FindPackageShare("perseus"),
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
    teleop_keyboard_controller = Node(
        package="perseus_teleop",
        executable="teleop_keyboard",
        name="teleop_keyboard",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        prefix="gnome-terminal --geometry=60x20 -- ",  # Use gnome-terminal instead of xterm
        remappings=[
            # Example: remap cmd_vel topic if needed
            # ('/cmd_vel_out', '/your_robot/cmd_vel')
        ],
    )

    # RViz with nixGL support
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

    # Define EKF node before it's used in TimerAction
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

    # Add delay to controllers
    controllers_delayed = TimerAction(
        period=15.0,  # Wait 15 seconds for Gazebo to fully start
        actions=[controllers_launch],
    )

    # Add delay to EKF to ensure all other nodes are ready
    ekf_delayed = TimerAction(
        period=5.0,  # Wait 5 seconds for robot_state_publisher and basic setup
        actions=[ekf_node],
    )

    # Fallback static transform publisher for odom->base_link (in case EKF fails)
    # This should not be needed if EKF is working properly
    fallback_odom_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="fallback_odom_publisher",
        arguments=[
            "0",
            "0",
            "0",  # translation x, y, z
            "0",
            "0",
            "0",
            "1",  # rotation quaternion x, y, z, w (identity - no rotation)
            "odom",  # parent frame
            "base_link",  # child frame
        ],
    )

    # Delay the fallback transform - only if EKF doesn't provide it
    fallback_transform_delayed = TimerAction(
        period=10.0,  # Wait 10 seconds - if EKF hasn't started by then, use fallback
        actions=[fallback_odom_transform],
    )

    # Static transform publisher for IMU frame to base_link - NOT NEEDED
    # The IMU sensor in Gazebo already publishes directly to base_link frame
    # static_transform_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='imu_to_base_link_publisher',
    #     arguments=[
    #         '0', '0', '0',  # translation x, y, z
    #         '0', '0', '0', '1',  # rotation quaternion x, y, z, w (identity - no rotation)
    #         'base_link',  # parent frame
    #         'imu_link'  # child frame - simplified name
    #     ]
    # )

    launch_files = [
        gz_launch,
        rsp_launch,  # Robot state publisher
        teleop_keyboard_controller,
        controllers_delayed,  # Controllers
        rviz,  # Start RViz with nixGL support
        ekf_delayed,  # EKF node with delay
        # fallback_transform_delayed,  # Uncomment if EKF fails to provide transform
        # static_transform_publisher not needed - IMU publishes directly to base_link
    ]

    return LaunchDescription(arguments + launch_files)

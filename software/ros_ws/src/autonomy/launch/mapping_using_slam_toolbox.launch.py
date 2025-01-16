from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ARGUMENTS
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    # use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")

    # declare_use_lifecycle_manager = DeclareLaunchArgument(
    #     "use_lifecycle_manager",
    #     default_value="false",
    #     description="Enable bond connection during node activation",
    # )

    use_legacy_hardware = LaunchConfiguration("use_legacy_hardware")
    can_bus = LaunchConfiguration("can_bus")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")

    arguments = [
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="False",
            description="Use mock hardware components which mirror commands to state interfaces",
        ),
        DeclareLaunchArgument(
            "use_legacy_hardware",
            default_value="True",
            description="Use legacy hardware (MCB) interfaces",
        ),
        DeclareLaunchArgument(
            "can_bus",
            default_value="can0",
            description="CAN bus to use for hardware communications",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation/Gazebo clock",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("autonomy"), "config", "slam_toolbox_params.yaml"]
            ),
            description="Full path to the ROS2 parameters file for SLAM Toolbox",
        ),
    ]

    # CONFIG FILES
    controller_config = PathJoinSubstitution(
        [FindPackageShare("perseus"), "config", "perseus_controllers.yaml"]
    )

    # XACRO FILES
    robot_description_xacro = PathJoinSubstitution(
        [FindPackageShare("perseus"), "urdf", "perseus.urdf.xacro"]
    )
    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                # pass through all the arguments
                " ",
                robot_description_xacro,
                " ",
                "use_mock_hardware:=",
                use_mock_hardware,
                " ",
                "use_legacy_hardware:=",
                use_legacy_hardware,
                " ",
                "can_bus:=",
                can_bus,
            ]
        ),
        value_type=str,
    )

    # NODES
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        output="both",  # output to both screen and log file
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/perseus_base_controller/cmd_vel", "/cmd_vel"),
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "robot_description": robot_description_content,
                "use_sim_time": use_sim_time,
                "publish_frequency": 30.0,
                "frame_prefix": "",  # Ensure no frame prefix is added
            }
        ],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "perseus_base_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    static_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_odom_publisher",
        # arguments=["0", "0", "0", "0", "0", "0", "base_link", "lidar_frame"],
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )
    # Node(
    # # Base link to LIDAR transform
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="lidar_tf_publisher",
    #     arguments=["0", "0", "0.2", "0", "0", "0", "odom", "lidar_frame"],
    # ),

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            #     get_package_share_directory("autonomy")
            #     + "/config/slam_toolbox_params.yaml",
            slam_params_file,
            # {
            #     "use_sim_time": use_sim_time,
            #     "odom_frame": "odom",
            #     "base_frame": "base_link",
            #     "map_frame": "map",
            #     "scan_topic": "/scan",
            # },
        ],
    )

    # RViz with nixGL support
    rviz_config_dir = PathJoinSubstitution(
        [FindPackageShare("autonomy"), "rviz", "perseus_slam.rviz"]
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
            rviz_config_dir,
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

    nodes = [
        robot_state_publisher,
        controller_manager,
        base_controller_spawner,
        joint_state_broadcaster_spawner,
        static_tf_publisher,
        slam_toolbox,
        rviz,
    ]

    # EVENT HANDLERS
    handlers = []

    # Return the launch description
    return LaunchDescription(arguments + nodes + handlers)

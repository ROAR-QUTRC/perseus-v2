from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ARGUMENTS
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    hardware_plugin = LaunchConfiguration("hardware_plugin")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    servo_max_rpm = LaunchConfiguration("servo_max_rpm")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    arguments = [
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
            "servo_max_rpm",
            default_value="112.8",
            description="Max RPM of STS3215 servos (62 at 7.4V, ~113 at 12V)",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel",
            description="Topic name for cmd_vel commands (use /joy_vel for xbox controller compatibility)",
        ),
    ]

    # IMPORTED LAUNCH FILES
    def robot_state_publisher(context):
        performed_use_mock_hardware = IfCondition(use_mock_hardware).evaluate(context)
        final_hardware_plugin = (
            "mock_components/GenericSystem"
            if performed_use_mock_hardware
            else hardware_plugin
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
                "hardware_plugin": final_hardware_plugin,
                "serial_port": serial_port,
                "baud_rate": baud_rate,
                "servo_max_rpm": servo_max_rpm,
            }.items(),
        )
        return [rsp_launch]

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
            "cmd_vel_topic": cmd_vel_topic,
        }.items(),
    )

    rplidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        name="rplidar_node",
        output="screen",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": "/dev/ttyUSB0",
                "serial_baudrate": 460800,
                "frame_id": "c1_lidar_frame",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Standard",
            }
        ],
    )

    i2c_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus_sensors"),
                        "launch",
                        "i2c_imu.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # INA228 DC power monitor (battery voltage, current, power)
    ina228_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus_sensors"),
                        "launch",
                        "ina228.launch.py",
                    ]
                )
            ]
        ),
    )

    # twist_mux to arbitrate between joystick and navigation cmd_vel sources
    twist_mux_config = PathJoinSubstitution(
        [FindPackageShare("autonomy"), "config", "twist_mux.yaml"]
    )
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_config, {"use_sim_time": use_sim_time}],
        remappings=[("/cmd_vel_out", "/cmd_vel")],
    )

    # Logitech C920 USB camera
    # Uses /dev/c920 symlink from udev rule (config/99-c920-camera.rules).
    # Falls back to /dev/video0 if the symlink is missing.
    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="v4l2_camera",
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        parameters=[
            {
                "video_device": "/dev/c920",
                "image_size": [640, 480],
                "pixel_format": "YUYV",
                "camera_frame_id": "camera_link",
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # Camera HUD overlay (compass, mini-map, LiDAR proximity, velocity, odometer)
    hud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus_lite_hud"),
                        "launch",
                        "hud.launch.py",
                    ]
                )
            ]
        ),
    )

    # Rosbridge WebSocket server (for AndroidRViz / web UI connections on port 9090)
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

    launch_files = [
        OpaqueFunction(function=robot_state_publisher),
        controllers_launch,
        rplidar_node,
        i2c_imu_launch,
        ina228_launch,
        camera_node,
        hud_launch,
        twist_mux_node,
        rosbridge_launch,
    ]

    return LaunchDescription(arguments + launch_files)

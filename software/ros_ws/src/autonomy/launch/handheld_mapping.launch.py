from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch.actions import GroupAction
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    autostart = LaunchConfiguration("autostart")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the slamtoolbox. "
        "Ignored when use_lifecycle_manager is true.",
    )
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        "use_lifecycle_manager",
        default_value="false",
        description="Enable bond connection during node activation",
    )

    # Robot localization node for sensor fusion
    start_robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            {
                # The frequency of the filter prediction step
                "frequency": 30.0,
                # Frame definitions
                "map_frame": "map",
                "odom_frame": "odom",
                "base_link_frame": "base_link",
                "world_frame": "odom",
                # True if sensor provides relative measurements
                "imu0_differential": False,
                "imu0_relative": True,
                # IMU sensor configuration
                "imu0": "/imu",
                "imu0_config": [
                    False,
                    False,
                    False,  # x, y, z position
                    False,
                    False,
                    False,  # roll, pitch, yaw position
                    True,
                    True,
                    True,  # x, y, z velocity
                    True,
                    True,
                    True,  # roll, pitch, yaw velocity
                    True,
                    True,
                    True,  # x, y, z acceleration
                ],
                # Laser scan odometry configuration
                "odom0": "/scan_matcher/odom",  # Output from laser scan matcher
                "odom0_config": [
                    True,
                    True,
                    False,  # x, y, z position
                    False,
                    False,
                    True,  # roll, pitch, yaw position
                    True,
                    True,
                    False,  # x, y, z velocity
                    False,
                    False,
                    True,  # roll, pitch, yaw velocity
                    False,
                    False,
                    False,  # x, y, z acceleration
                ],
                "odom0_differential": True,
                # Whether to publish the acceleration state
                "publish_acceleration": False,
            }
        ],
    )

    start_lifelong_slam_toolbox_node = LifecycleNode(
        parameters=[
            get_package_share_directory("slam_toolbox")
            + "/config/mapper_params_lifelong.yaml",
            {"use_lifecycle_manager": use_lifecycle_manager},
        ],
        package="slam_toolbox",
        executable="lifelong_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        namespace="",
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_lifelong_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_lifelong_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(
                            start_lifelong_slam_toolbox_node
                        ),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    ld = LaunchDescription()

    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)

    # Group localization nodes together
    localization_group = GroupAction(
        [
            # Start laser scan matcher first to provide odometry
            start_laser_scan_matcher_node,
            # Then robot_localization to fuse with IMU
            start_robot_localization_node,
        ]
    )

    ld.add_action(localization_group)
    ld.add_action(start_lifelong_slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)

    return ld

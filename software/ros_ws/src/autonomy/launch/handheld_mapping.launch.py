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

    # # Robot localization node for sensor fusion
    # start_robot_localization_node = Node(
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="ekf_filter_node",
    #     output="screen",
    #     parameters=[
    #         {
    #             # The frequency of the filter prediction step
    #             "frequency": 30.0,
    #             # Frame definitions
    #             "publish_tf": True,
    #             "map_frame": "map",
    #             "odom_frame": "odom",
    #             "base_link_frame": "base_link",
    #             "world_frame": "odom",
    #             "imu_frame": "imu_link",
    #             # True if sensor provides relative measurements
    #             "imu0_differential": False,
    #             "imu0_relative": False,
    #             # IMU sensor configuration
    #             "imu0": "/imu",
    #             "imu0_pose_rejection_threshold": 0.0,
    #             "imu0_twist_rejection_threshold": 0.0,
    #             "imu0_linear_acceleration_rejection_threshold": 0.0,
    #             "ignore_orientation_covariance": True,
    #             "ignore_angular_velocity_covariance": True,
    #             "ignore_linear_acceleration_covariance": True,
    #             "imu0_config": [
    #                 False,
    #                 False,
    #                 False,  # x, y, z position
    #                 True,
    #                 True,
    #                 True,  # roll, pitch, yaw
    #                 False,
    #                 False,
    #                 False,  # x, y, z velocity
    #                 True,
    #                 True,
    #                 True,  # roll, pitch, yaw rates
    #                 True,
    #                 True,
    #                 True,  # x, y, z accelerations
    #             ],
    #             # Whether to publish the acceleration state
    #             "imu0_remove_gravitational_acceleration": True,
    #             "imu0_queue_size": 10,
    #             "imu0_nodelay": True,
    #             "imu_frame": "lidar_frame",  # Match the actual IMU frame_id
    #             "publish_acceleration": False,
    #             "print_diagnostics": True,
    #             "debug": True,
    #             "debug_out_file": "robot_localization_debug.txt",
    #             "use_fixed_covariance": True,
    #             "imu0_pose_covariance": [
    #                 0.01,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.01,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.01,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.01,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.01,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.0,
    #                 0.01,
    #             ],
    #         }
    #     ],
    # )

    start_lifelong_slam_toolbox_node = LifecycleNode(
        parameters=[
            get_package_share_directory("autonomy")
            + "/config/slam_toolbox_params.yaml",
            {"use_lifecycle_manager": use_lifecycle_manager},
        ],
        package="slam_toolbox",
        # executable="lifelong_slam_toolbox_node",
        executable="async_slam_toolbox_node",
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

    # Define all static transforms for the robot
    static_transform_nodes = [
        # Base link to LIDAR transform
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="odom_tf_publisher",
            arguments=["0", "0", "0.0", "0", "0", "0", "odom", "base_footprint"],
        ),
        # Base link to IMU transform
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_tf_publisher",
            arguments=["0", "0", "0.1", "0", "0", "0", "base_footprint", "laser_frame"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_tf_publisher",
            arguments=["0", "0", "0.4", "0", "0", "0", "base_footprint", "imu_frame"],
        ),
        # Add diagnostic node to check TF timing
        Node(
            package="tf2_ros",
            executable="tf2_monitor",
            name="tf_monitor",
            output="screen",
        ),
    ]

    ld = LaunchDescription()

    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)

    # Group localization nodes together
    localization_group = GroupAction(
        [
            # First add all static transforms
            *static_transform_nodes,
            # Then robot_localization to fuse with IMU
            # start_robot_localization_node,
        ]
    )

    ld.add_action(localization_group)
    ld.add_action(start_lifelong_slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)

    return ld

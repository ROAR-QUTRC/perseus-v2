from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
<<<<<<< HEAD
=======
    GroupAction,
>>>>>>> origin/main
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import LifecycleNode, Node
<<<<<<< HEAD
from launch.actions import GroupAction
=======
>>>>>>> origin/main
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
<<<<<<< HEAD
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

=======
    # ARGUMENTS
    autostart = LaunchConfiguration("autostart")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")

    arguments = [
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically startup the slamtoolbox. "
            "Ignored when use_lifecycle_manager is true.",
        ),
        DeclareLaunchArgument(
            "use_lifecycle_manager",
            default_value="false",
            description="Enable bond connection during node activation",
        ),
    ]

    # NODES
>>>>>>> origin/main
    start_lifelong_slam_toolbox_node = LifecycleNode(
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("autonomy"), "config", "slam_toolbox_params.yaml"]
            ),
            {"use_lifecycle_manager": use_lifecycle_manager},
        ],
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        namespace="",
    )

<<<<<<< HEAD
=======
    # EVENTS
>>>>>>> origin/main
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
            arguments=["0", "0", "0.0", "0", "0", "0", "odom", "base_link"],
        ),
        # Base link to IMU transform
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_tf_publisher",
            arguments=["0", "0", "0.1", "0", "0", "0", "base_link", "laser_frame"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_tf_publisher",
            arguments=["0", "0", "0.4", "0", "0", "0", "base_link", "imu_frame"],
        ),
        # Add diagnostic node to check TF timing
        Node(
            package="tf2_ros",
            executable="tf2_monitor",
            name="tf_monitor",
            output="screen",
        ),
    ]

<<<<<<< HEAD
    ld = LaunchDescription()

    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)

=======
>>>>>>> origin/main
    # Group localization nodes together
    localization_group = GroupAction(
        [
            # First add all static transforms
            *static_transform_nodes,
            # Then robot_localization to fuse with IMU
            # start_robot_localization_node,
        ]
    )

<<<<<<< HEAD
    ld.add_action(localization_group)
    ld.add_action(start_lifelong_slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)

    return ld
=======
    # Combine all launch elements
    launch_elements = [
        *arguments,
        localization_group,
        start_lifelong_slam_toolbox_node,
        configure_event,
        activate_event,
    ]

    return LaunchDescription(launch_elements)
>>>>>>> origin/main

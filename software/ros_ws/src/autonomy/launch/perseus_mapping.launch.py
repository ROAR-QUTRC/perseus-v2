from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
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
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Launch configurations
    autostart = LaunchConfiguration("autostart")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")

    # Declare launch arguments as variables
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.",
    )

    declare_use_lifecycle_manager = DeclareLaunchArgument(
        "use_lifecycle_manager",
        default_value="false",
        description="Enable bond connection during node activation",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation/Gazebo clock"
    )

    declare_slam_params_file = DeclareLaunchArgument(
        "slam_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("slam_toolbox"), "config", "slam_toolbox_params.yaml"]
        ),
        description="Full path to the ROS2 parameters file for slam_toolbox",
    )

    # SLAM Toolbox Node as variable
    slam_toolbox_node = LifecycleNode(
        parameters=[
            slam_params_file,
            {
                "use_lifecycle_manager": use_lifecycle_manager,
                "use_sim_time": use_sim_time,
            },
        ],
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        namespace="",
    )

    # Lifecycle events as variables
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    # TF Monitor as variable
    tf_monitor_node = Node(
        package="tf2_ros", executable="tf2_monitor", name="tf_monitor", output="screen"
    )

    # Build LaunchDescription
    ld = LaunchDescription()

    # Add actions using variables
    ld.add_action(declare_autostart)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_slam_params_file)
    ld.add_action(slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)
    ld.add_action(tf_monitor_node)

    return ld

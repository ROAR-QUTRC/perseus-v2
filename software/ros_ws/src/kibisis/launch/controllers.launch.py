from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=False)
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic", default="/cmd_vel")
    use_sim_time_param = {"use_sim_time": use_sim_time}

    controller_config = PathJoinSubstitution(
        [FindPackageShare("kibisis"), "config", "kibisis_controllers.yaml"]
    )

    # -------------------------------------------------------------------------
    # Controller manager — owns the hardware interface and I2C bus.
    # Namespaced under /kibisis to avoid collision with perseus.
    # -------------------------------------------------------------------------
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="kibisis",
        name="kibisis_control_manager",
        parameters=[controller_config, use_sim_time_param],
        output="both",
        remappings=[
            ("/kibisis/robot_description", "/kibisis/robot_description"),
        ],
    )

    # -------------------------------------------------------------------------
    # Controller spawners — all reference the namespaced controller manager
    # -------------------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/kibisis/kibisis_control_manager",
            "--controller-manager-timeout", "30",
        ],
        parameters=[use_sim_time_param],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_base_controller",
            "--controller-manager", "/kibisis/kibisis_control_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
        parameters=[use_sim_time_param],
        remappings=[
            ("/diff_drive_base_controller/cmd_vel",
             ["/kibisis", cmd_vel_topic]),
        ],
    )

    space_motor_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "space_motor_controller",
            "--controller-manager", "/kibisis/kibisis_control_manager",
            "--controller-manager-timeout", "30",
        ],
        parameters=[use_sim_time_param],
    )

    moisture_trigger_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "moisture_trigger_controller",
            "--controller-manager", "/kibisis/kibisis_control_manager",
            "--controller-manager-timeout", "30",
        ],
        parameters=[use_sim_time_param],
    )

    # -------------------------------------------------------------------------
    # kibisis_sensors node — bridges ROS2 API to GPIO interfaces.
    # Namespaced under /kibisis so all its topics are /kibisis/moisture/...
    # and /kibisis/space_motor/...
    # -------------------------------------------------------------------------
    kibisis_sensors = Node(
        package="kibisis_sensors",
        executable="kibisis_sensors_node",
        name="kibisis_sensors",
        namespace="kibisis",
        output="screen",
        parameters=[use_sim_time_param],
    )

    # -------------------------------------------------------------------------
    # Launch sequence:
    #   1. controller_manager + joint_state_broadcaster
    #   2. diff_drive + space_motor + moisture_trigger (after jsb is up)
    #   3. kibisis_sensors (after moisture_trigger is up)
    # -------------------------------------------------------------------------
    return LaunchDescription([
        controller_manager,
        joint_state_broadcaster_spawner,

        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
                    diff_drive_spawner,
                    space_motor_spawner,
                    moisture_trigger_spawner,
                ],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=moisture_trigger_spawner,
                on_exit=[kibisis_sensors],
            )
        ),
    ])

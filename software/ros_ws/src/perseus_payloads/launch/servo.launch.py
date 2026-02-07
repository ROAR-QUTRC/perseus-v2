from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Use mock hardware (simulation) instead of real CAN hardware",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_bus",
            default_value="vcan0",
            description="CAN interface to use (e.g., can0, vcan0)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz2 for visualization",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_servo",
            default_value="true",
            description="Launch MoveIt Servo for real-time teleoperation",
        )
    )

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    can_bus = LaunchConfiguration("can_bus")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_servo = LaunchConfiguration("use_servo")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("perseus_payloads"), "config", "arm.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("perseus_payloads"), "config", "arm.srdf"]
            ),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }

    kinematics_yaml = load_yaml("perseus_payloads", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    servo_yaml = load_yaml("perseus_payloads", "config/servo.yaml")

    joint_limits_yaml = load_yaml("perseus_payloads", "config/joint_limits.yaml")
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}

    ompl_planning_yaml = load_yaml("perseus_payloads", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
            ],
            "start_state_max_bounds_error": 0.1,
        },
    }
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    moveit_controllers = {
        "moveit_simple_controller_manager": load_yaml(
            "perseus_payloads", "config/moveit_controllers.yaml"
        ),
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "use_sim_time": use_sim_time,
        "joint_state_topic": "/joint_states",
    }

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # 2. ros2_control Node 
    ros2_controllers_path = os.path.join(
        get_package_share_directory("perseus_payloads"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers_path,
            {"use_sim_time": use_sim_time},
        ],
        output="both",
    )

    # 3. Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    servo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["servo_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 4. RSBL Driver Node 
    rsbl_driver_node = Node(
        package="perseus_payloads",
        executable="rsbl_driver",
        name="rsbl_servo_driver",
        parameters=[{"can_bus": can_bus}],
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
    )

    # 5. Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    # 6. MoveIt Servo Node 
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            {"moveit_servo": servo_yaml},
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
        condition=IfCondition(use_servo),
    )

    # 7. RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("perseus_payloads"), "config", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_rviz),
    )

    # 8. Static Transform Publisher (world -> plate)
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "plate"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    delayed_rsbl_driver = TimerAction(
        period=2.0,
        actions=[rsbl_driver_node],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        servo_controller_spawner,
        delayed_rsbl_driver,
        move_group_node,
        servo_node,
        rviz_node,
        static_tf_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)

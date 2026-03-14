from launch import LaunchDescription, Substitution, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from typing import Text


class ConditionalText(Substitution):
    def __init__(self, text_if, text_else, condition):
        self.text_if = text_if
        self.text_else = text_else
        self.condition = condition

    def perform(self, context: "LaunchContext") -> Text:
        if self.condition is True or self.condition in ("true", "True"):
            return self.text_if
        else:
            return self.text_else


def launch_setup(context, *args, **kwargs):

    return [
        DeclareLaunchArgument(
            "depth",
            default_value=ConditionalText(
                "false",
                "true",
                IfCondition(
                    PythonExpression(
                        ["'", LaunchConfiguration("stereo"), "' == 'true'"]
                    )
                )._predicate_func(context),
            ),
            description="",
        ),
        DeclareLaunchArgument(
            "subscribe_rgb", default_value=LaunchConfiguration("depth"), description=""
        ),
        DeclareLaunchArgument(
            "rtabmap_args",
            default_value="--delete_db_on_start",
            description='Backward compatibility, use "args" instead.',
        ),
        DeclareLaunchArgument(
            "args",
            default_value=LaunchConfiguration("rtabmap_args"),
            description="Can be used to pass RTAB-Map's parameters or other flags like --udebug and --delete_db_on_start/-d",
        ),
        DeclareLaunchArgument(
            "sync_queue_size",
            default_value=LaunchConfiguration("queue_size"),
            description="Queue size of topic synchronizers.",
        ),
        DeclareLaunchArgument(
            "qos_image",
            default_value=LaunchConfiguration("qos"),
            description="Specific QoS used for image input data: 0=system default, 1=Reliable, 2=Best Effort.",
        ),
        DeclareLaunchArgument(
            "qos_camera_info",
            default_value=LaunchConfiguration("qos"),
            description="Specific QoS used for camera info input data: 0=system default, 1=Reliable, 2=Best Effort.",
        ),
        DeclareLaunchArgument(
            "qos_scan",
            default_value=LaunchConfiguration("qos"),
            description="Specific QoS used for scan input data: 0=system default, 1=Reliable, 2=Best Effort.",
        ),
        DeclareLaunchArgument(
            "qos_odom",
            default_value=LaunchConfiguration("qos"),
            description="Specific QoS used for odometry input data: 0=system default, 1=Reliable, 2=Best Effort.",
        ),
        DeclareLaunchArgument(
            "qos_user_data",
            default_value=LaunchConfiguration("qos"),
            description="Specific QoS used for user input data: 0=system default, 1=Reliable, 2=Best Effort.",
        ),
        DeclareLaunchArgument(
            "qos_imu",
            default_value=LaunchConfiguration("qos"),
            description="Specific QoS used for imu input data: 0=system default, 1=Reliable, 2=Best Effort.",
        ),
        DeclareLaunchArgument(
            "qos_gps",
            default_value=LaunchConfiguration("qos"),
            description="Specific QoS used for gps input data: 0=system default, 1=Reliable, 2=Best Effort.",
        ),
        DeclareLaunchArgument(
            "qos_env_sensor",
            default_value=LaunchConfiguration("qos"),
            description="Specific QoS used for env sensor input data: 0=system default, 1=Reliable, 2=Best Effort.",
        ),
        DeclareLaunchArgument(
            "odom_log_level",
            default_value=LaunchConfiguration("log_level"),
            description="Specific ROS logger level for odometry node.",
        ),

        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),

        Node(
            package="rtabmap_odom",
            executable="stereo_odometry",
            name="stereo_odometry",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        LaunchConfiguration("icp_odometry"),
                        "' != 'true' and '",
                        LaunchConfiguration("visual_odometry"),
                        "' == 'true' and '",
                        LaunchConfiguration("stereo"),
                        "' == 'true'",
                    ]
                )
            ),
            parameters=[
                {
                    "frame_id": LaunchConfiguration("frame_id"),
                    "odom_frame_id": LaunchConfiguration("vo_frame_id"),
                    "publish_tf": LaunchConfiguration("publish_tf_odom"),
                    "wait_for_transform": LaunchConfiguration("wait_for_transform"),
                    "approx_sync": LaunchConfiguration("approx_sync"),
                    "approx_sync_max_interval": LaunchConfiguration(
                        "approx_sync_max_interval"
                    ),
                    "topic_queue_size": LaunchConfiguration("topic_queue_size"),
                    "sync_queue_size": LaunchConfiguration("sync_queue_size"),
                    "qos": LaunchConfiguration("qos_image"),
                    "qos_camera_info": LaunchConfiguration("qos_camera_info"),
                    "qos_imu": LaunchConfiguration("qos_imu"),
                }
            ],
            remappings=[
                ("left/image_rect", LaunchConfiguration("left_image_topic")),
                ("right/image_rect", LaunchConfiguration("right_image_topic")),
                ("left/camera_info", LaunchConfiguration("left_camera_info_topic")),
                ("right/camera_info", LaunchConfiguration("right_camera_info_topic")),
                ("odom", LaunchConfiguration("odom_topic")),
                ("imu", LaunchConfiguration("imu_topic")),
            ],
            namespace=LaunchConfiguration("namespace"),
        ),
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "stereo",
                default_value="true",
                description="Use stereo input instead of RGB-D.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS logging level (debug, info, warn, error).",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="base_link",
                description="Fixed frame id of the robot (base frame).",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="rtabmap",
                description="Namespace for odometry nodes.",
            ),
            DeclareLaunchArgument(
                "topic_queue_size",
                default_value="5",
                description="Queue size of individual topic subscribers.",
            ),
            DeclareLaunchArgument(
                "queue_size",
                default_value="5",
                description='Backward compatibility, use "sync_queue_size" instead.',
            ),
            DeclareLaunchArgument(
                "qos",
                default_value="0",
                description="General QoS used for sensor input data.",
            ),
            DeclareLaunchArgument(
                "wait_for_transform",
                default_value="0.2",
                description="Wait for transform timeout.",
            ),

            DeclareLaunchArgument(
                "stereo_namespace",
                default_value="/camera/camera",
                description=""
            ),

            DeclareLaunchArgument(
                "left_image_topic",
                default_value="/camera/camera/infra2/image_raw",
                description="",
            ),

            DeclareLaunchArgument(
                "right_image_topic",
                default_value="/camera/camera/infra1/image_raw",
                description="Use grayscale image for efficiency",
            ),

            DeclareLaunchArgument(
                "left_camera_info_topic",
                default_value="/camera/camera/infra2/camera_info",
                description="",
            ),

            DeclareLaunchArgument(
                "right_camera_info_topic",
                default_value="/camera/camera/infra1/camera_info",
                description="",
            ),

            DeclareLaunchArgument(
                "visual_odometry",
                default_value="true",
                description="Launch rtabmap visual odometry node.",
            ),

            DeclareLaunchArgument(
                "icp_odometry",
                default_value="false",
                description="Launch rtabmap icp odometry node.",
            ),

            DeclareLaunchArgument(
                "odom_topic", default_value="odom", description="Odometry topic name."
            ),

            DeclareLaunchArgument(
                "vo_frame_id",
                default_value=LaunchConfiguration("odom_topic"),
                description="Visual/Icp odometry frame ID for TF.",
            ),

            DeclareLaunchArgument(
                "publish_tf_odom",
                default_value="true",
                description="Publish TF frames for odometry.",
            ),

            DeclareLaunchArgument(
                "imu_topic",
                default_value="/imu/data",
                description="IMU topic for odometry.",
            ),

            DeclareLaunchArgument(
                "approx_sync",
                default_value="true",
                description="Approximate synchronization of sensor topics.",
            ),

            DeclareLaunchArgument(
                "approx_sync_max_interval",
                default_value="0.0",
                description="Maximum interval for approximate synchronization.",
            ),

            OpaqueFunction(function=launch_setup),
        ]
    )

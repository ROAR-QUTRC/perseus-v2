"""Launch a V4L2 webcam as a drop-in image source for the perseus_vision detectors."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for the webcam capture node."""
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulated time"
    )
    video_device_arg = DeclareLaunchArgument(
        "video_device",
        default_value="/dev/video0",
        description="V4L2 device to capture from (see `ls /dev/video*`)",
    )
    image_width_arg = DeclareLaunchArgument(
        "image_width", default_value="640", description="Captured image width in pixels"
    )
    image_height_arg = DeclareLaunchArgument(
        "image_height",
        default_value="480",
        description="Captured image height in pixels",
    )
    # Output topics default to the RealSense colour topics so the webcam is a
    # drop-in source for aruco_detector / cube_detector.
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/camera/camera/color/image_raw",
        description="Topic to publish the raw image on",
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera/camera/color/camera_info",
        description="Topic to publish the camera_info on",
    )
    camera_frame_arg = DeclareLaunchArgument(
        "camera_frame",
        default_value="camera_color_optical_frame",
        description="frame_id stamped on published images",
    )

    # v4l2_camera node: captures from a V4L2 device and publishes image_raw + camera_info
    webcam_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="webcam",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "video_device": LaunchConfiguration("video_device"),
                "image_size": [
                    LaunchConfiguration("image_width"),
                    LaunchConfiguration("image_height"),
                ],
                "camera_frame_id": LaunchConfiguration("camera_frame"),
                "output_encoding": "rgb8",
            }
        ],
        remappings=[
            ("image_raw", LaunchConfiguration("image_topic")),
            ("camera_info", LaunchConfiguration("camera_info_topic")),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            video_device_arg,
            image_width_arg,
            image_height_arg,
            image_topic_arg,
            camera_info_topic_arg,
            camera_frame_arg,
            webcam_node,
        ]
    )

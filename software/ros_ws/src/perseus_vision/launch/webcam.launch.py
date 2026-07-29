"""Launch a V4L2 webcam, the detection overlay, and rviz as one viewable pipeline.

The webcam is a drop-in replacement for the RealSense colour stream, so the detectors
can be launched separately against the same topics. The overlay is included here because
it republishes the camera image as bgr8, which rviz can display regardless of the
camera's native encoding.
"""

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the launch description for the webcam, overlay, and rviz."""
    perseus_vision_dir = get_package_share_directory("perseus_vision")
    config_file = os.path.join(perseus_vision_dir, "config", "perseus_vision.yaml")

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
    # Publishing the camera's native format avoids a conversion inside v4l2_camera,
    # whose YUYV -> RGB path is a naive per-pixel loop costing ~200 ms per 640x480 frame.
    # That cost caps the frame rate and, because frames are stamped at capture, shows up
    # as latency too. cv_bridge in the detectors converts once instead, via OpenCV.
    # Set to "rgb8" only if a consumer cannot handle yuv422_yuy2.
    output_encoding_arg = DeclareLaunchArgument(
        "output_encoding",
        default_value="yuv422_yuy2",
        description="Encoding to publish images in; native avoids a slow driver conversion",
    )
    camera_frame_arg = DeclareLaunchArgument(
        "camera_frame",
        default_value="camera_color_optical_frame",
        description="frame_id stamped on published images",
    )
    use_overlay_arg = DeclareLaunchArgument(
        "use_overlay",
        default_value="true",
        description="Run the detection overlay, which republishes the image as bgr8",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Open rviz to view the overlay"
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
                # A plain Python list here would be concatenated into a single
                # substitution ("640" + "480" = "640480") and offered to the node as one
                # integer, which it rejects. Build the YAML list text and declare the
                # type explicitly so it arrives as an integer array.
                "image_size": ParameterValue(
                    [
                        "[",
                        LaunchConfiguration("image_width"),
                        ",",
                        LaunchConfiguration("image_height"),
                        "]",
                    ],
                    value_type=List[int],
                ),
                "camera_frame_id": LaunchConfiguration("camera_frame"),
                "output_encoding": LaunchConfiguration("output_encoding"),
            }
        ],
        remappings=[
            ("image_raw", LaunchConfiguration("image_topic")),
            ("camera_info", LaunchConfiguration("camera_info_topic")),
        ],
        output="screen",
    )

    # Republishes the camera image annotated with whatever detections have arrived. Also
    # normalises the encoding to bgr8, so rviz can display it even when the camera
    # publishes a native format rviz does not understand.
    overlay_node = Node(
        package="perseus_vision",
        executable="detection_overlay_node",
        name="detection_overlay",
        parameters=[
            config_file,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "input_image_topic": LaunchConfiguration("image_topic"),
            },
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_overlay")),
    )

    # rviz must be run through nixGL so it can reach the host GPU driver; plain rviz2
    # from nix dies with "Unable to create glx visual". nixGL is invoked directly here
    # rather than via the repo's rviz2-fixed wrapper, as this is the form that works.
    # ExecuteProcess is used because this is a command line, not a ROS package executable.
    # No -d config is passed: rviz opens with its default view, so add an Image display on
    # the overlay topic by hand.
    rviz = ExecuteProcess(
        cmd=[
            "nix",
            "run",
            "--impure",
            "github:nix-community/nixGL",
            "--",
            "rviz2",
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
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            video_device_arg,
            image_width_arg,
            image_height_arg,
            image_topic_arg,
            camera_info_topic_arg,
            output_encoding_arg,
            camera_frame_arg,
            use_overlay_arg,
            use_rviz_arg,
            rviz_config_arg,
            webcam_node,
            overlay_node,
            rviz,
        ]
    )

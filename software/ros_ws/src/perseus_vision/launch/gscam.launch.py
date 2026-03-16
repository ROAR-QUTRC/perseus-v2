from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    IfElseSubstitution,
    EqualsSubstitution,
    NotEqualsSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_name = LaunchConfiguration("camera_name")
    camera_config = LaunchConfiguration("gst_config")
    camera_info = LaunchConfiguration("camera_info")
    camera_namespace = LaunchConfiguration("camera_namespace")
    jpeg_encoding = LaunchConfiguration("jpeg_encoding")

    arguments = [
        DeclareLaunchArgument(
            "camera_info",
            default_value="logitech.yaml",
            description="The name of the yaml or ini file to be parsed by the camera_calibration_parsers package - inside perseus_vision/config/",
        ),
        DeclareLaunchArgument(
            "gst_config",
            default_value="",
            description="The gstreamer pipeline to be run to get the camera stream",
        ),
        DeclareLaunchArgument(
            "camera_name",
            default_value="",
            description="The camera device name to stream from (in /dev/v4l/by-id/). If not specified, /dev/video0 will be used",
        ),
        DeclareLaunchArgument(
            "camera_namespace",
            default_value="",
            description="The namespace to prepend to any topics/services published from the gscam node. If not specified, the camera name will be used.",
        ),
        DeclareLaunchArgument(
            "jpeg_encoding",
            default_value="true",
            description="Whether to encode the image as a compressed jpeg file",
        ),
    ]

    camera_info_path = [
        "file://",
        PathJoinSubstitution(
            [FindPackageShare("perseus_vision"), "config", camera_info]
        ),
    ]

    camera_filepath = IfElseSubstitution(
        EqualsSubstitution(camera_name, ""),
        "/dev/video0",
        ["/dev/v4l/by-id/", camera_name],
    )

    # TODO: Replace this with StringJoinSubstitution when we upgrade ROS versions
    default_camera_config = [
        "v4l2src device=",
        camera_filepath,
        " ! video/x-raw",
        " ! videoconvert",
    ]
    pipeline_with_encoding = IfElseSubstitution(
        jpeg_encoding, default_camera_config + [" ! jpegenc"], default_camera_config
    )

    gst_pipeline = IfElseSubstitution(
        EqualsSubstitution(camera_config, ""), pipeline_with_encoding, camera_config
    )

    gscam_namespace = IfElseSubstitution(
        NotEqualsSubstitution(camera_namespace, ""),
        camera_namespace,
        IfElseSubstitution(
            NotEqualsSubstitution(camera_name, ""),
            PythonExpression(
                [
                    "'",
                    camera_name,
                    "'.replace('-', '_')",
                    ".replace('.', '')",
                ]  # Why can't I write python in a python file!!!
            ),
            "/",
        ),
    )
    image_encoding = IfElseSubstitution(jpeg_encoding, "jpeg", "rgb8")
    gscam_node = Node(
        package="gscam",
        executable="gscam_node",
        namespace=gscam_namespace,
        output="both",
        parameters=[
            {
                "reopen_on_eof": True,
                "gscam_config": gst_pipeline,
                "camera_info_url": camera_info_path,
                "image_encoding": image_encoding,
            }
        ],
    )

    nodes = [gscam_node]

    return LaunchDescription(arguments + nodes)

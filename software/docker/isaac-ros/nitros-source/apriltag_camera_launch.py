"""
Stage 4 live-camera AprilTag test: usb_cam -> apriltag directly, using our
own camera_c920_params.yaml / camera_c920_info.yaml (see those files for why
-- NVIDIA's own isaac_ros_apriltag_usb_cam.launch.py + usb_cam_params.yaml
has a resolution/calibration mismatch: image_raw defaults to 640x480 but
their bundled camera_info.yaml is calibrated for 1280x720).

NOT the same graph as isaac_ros_apriltag_usb_cam.launch.py -- that one
chains isaac_ros_image_proc::RectifyNode in front of AprilTagNode. Skipped
here: rectify->apriltag on live/real frames reproducibly crashes
("terminate called after throwing an instance of 'nvcv::Exception': ...
The tensor handle is null.") right after apriltag finishes loading. Isolated
to specifically that combination -- apriltag alone works fine (see
apriltag_check.py, apriltag_launch.py), and rectify+apriltag loaded together
with NO live frames flowing also loads fine; it's rectify actively
processing real camera frames while apriltag initializes/runs that trips
it. Not yet root-caused -- see isaac-ros-nitros-source-build.md. Detection
without rectification means the C920's lens distortion isn't corrected
(pose numbers less accurate; mild for a webcam, but real), which is an
acceptable trade for now to get live detection actually running.
"""
import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

THIS_DIR = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    usb_cam_params_path = os.path.join(THIS_DIR, 'camera_c920_params.yaml')
    camera_info_path = os.path.join(THIS_DIR, 'camera_c920_info.yaml')

    usb_cam_node = ComposableNode(
        package='usb_cam',
        plugin='usb_cam::UsbCamNode',
        name='usb_cam',
        parameters=[
            usb_cam_params_path,
            {'camera_info_url': f'file://{camera_info_path}'},
        ],
    )

    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace='',
        parameters=[{'size': 0.22, 'max_tags': 64, 'tile_size': 4}],
        remappings=[('image', 'image_raw')],  # camera_info already matches
    )

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[usb_cam_node, apriltag_node],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )
    return LaunchDescription([container])

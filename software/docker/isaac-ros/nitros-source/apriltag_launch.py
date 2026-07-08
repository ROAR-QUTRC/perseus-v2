"""
Stage 4: minimal AprilTag node launch, matching the parameters NVIDIA's own
isaac_ros_apriltag_pol_test.py uses (size=0.22, tag36h11 family default).
Our own minimal launch file, not NVIDIA's.
"""
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace='',
        parameters=[{'size': 0.22, 'max_tags': 64, 'tile_size': 4}],
    )
    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[apriltag_node],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )
    return LaunchDescription([container])

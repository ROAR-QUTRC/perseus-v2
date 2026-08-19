"""Launch the point cloud decoder node, for local RViz visualization of the downsampled cloud."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for the point cloud decoder node."""
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulated time"
    )

    point_cloud_decoder_node = Node(
        package="perseus_sensors",
        executable="point_cloud_decoder",
        name="point_cloud_decoder",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            point_cloud_decoder_node,
        ]
    )

"""
Topic Remapper Launch File (Python version)

This launch file starts the topic_remapper node with configurable parameters.

Usage:
    ros2 launch perseus_sensors topic_remapper_launch.py
    
    With custom parameters:
    ros2 launch perseus_sensors topic_remapper_launch.py \
        input_topic:=/scan \
        output_topic:=/scan_remapped \
        reduction_frequency:=10.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for topic_remapper node."""

    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        "input_topic",
        default_value="/livox/imu",
        description="Input topic name to remap",
    )

    output_topic_arg = DeclareLaunchArgument(
        "output_topic",
        default_value="/livox/imu_remapped",
        description="Output topic name",
    )

    reduction_frequency_arg = DeclareLaunchArgument(
        "reduction_frequency",
        default_value="50.0",
        description="Target output frequency in Hz",
    )

    # Get launch configuration
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    reduction_frequency = LaunchConfiguration("reduction_frequency")

    # Create topic_remapper node
    topic_remapper_node = Node(
        package="perseus_sensors",
        executable="topic_remapper",
        name="topic_remapper",
        output="screen",
        parameters=[
            {"input_topic": input_topic},
            {"output_topic": output_topic},
            {"reduction_frequency": reduction_frequency},
        ],
    )

    # Create launch description and populate
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(input_topic_arg)
    ld.add_action(output_topic_arg)
    ld.add_action(reduction_frequency_arg)

    # Add nodes
    ld.add_action(topic_remapper_node)

    return ld

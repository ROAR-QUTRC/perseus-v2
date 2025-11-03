from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ARGUMENTS
    arguments = []

    # NODES
    bucket_controller_node = Node(
        package="perseus_payloads",
        executable="bucket_controller",
        name="bucket_controller",
        output="both",
        emulate_tty=True,
    )
    bucket_driver_node = Node(
        package="perseus_payloads",
        executable="bucket_driver",
        name="bucket_driver",
        output="both",
        emulate_tty=True,
    )
    nodes = [bucket_controller_node, bucket_driver_node]

    return LaunchDescription(arguments + nodes)

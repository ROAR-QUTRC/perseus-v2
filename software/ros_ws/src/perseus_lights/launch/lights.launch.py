from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    # NODES
    light_driver = Node(
        package="perseus_lights",
        executable="light_driver",
        output="both",
    )
    light_controller = Node(
        package="perseus_lights",
        executable="light_controller",
        output="both",
    )
    light_orchestrator = Node(
        package="perseus_lights",
        executable="light_orchestrator",
        output="both",
    )

    nodes = [light_driver, light_controller]

    return LaunchDescription(nodes)

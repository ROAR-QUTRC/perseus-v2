from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # NODES
    centrifuge_driver = Node(
        package='perseus_payloads',
        executable='centrifuge_driver',
        name='centrifuge_driver',
        output='screen'
    )
    ilmenite_mechanics = Node(
        package='perseus_payloads',
        executable='ilmenite_mechanics',
        name='ilmenite_mechanics',
        output='screen'
    )
    ilmenite_sensor = Node(
        package='perseus_payloads',
        executable='ilmenite_sensor',
        name='ilmenite_sensor',
        output='screen'
    )
    space_resources_controller = Node(
        package='perseus_payloads',
        executable='space_resources_controller',
        name='space_resources_controller',
        output='screen'
    )

    # List of nodes
    nodes = [
        centrifuge_driver,
        ilmenite_mechanics,
        ilmenite_sensor,
        space_resources_controller
    ]

    return LaunchDescription(nodes)

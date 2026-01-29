from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    centrifuge_driver = Node(
        package="perseus_payloads",
        executable="centrifuge_driver",
        output="both",
    )

    ilmenite_station = Node(
        package="perseus_payloads",
        executable="ilmenite_station",
        output="both",
    )

    ilmenite_ml = Node(
        package="perseus_payloads",
        executable="ilmenite_ml",
        output="both",
    )

    space_resources_controller = Node(
        package="perseus_payloads",
        executable="space_resources_controller",
        output="both",
    )

    nodes = [
        centrifuge_driver,
        ilmenite_station,
        ilmenite_ml,
        space_resources_controller,
    ]

    return LaunchDescription(nodes)

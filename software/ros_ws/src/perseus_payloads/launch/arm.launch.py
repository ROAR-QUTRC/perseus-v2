from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    arm_controller = Node(
        package="perseus_payloads",
        executable="arm_controller",
        output="both",
        parameters=[{"can_bus": "can0"}]
    )

    return LaunchDescription([arm_controller])

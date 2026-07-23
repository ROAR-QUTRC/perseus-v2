from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    arm_driver = Node(
        package="perseus_payloads",
        executable="arm_driver",
        output="both",
    )
    return LaunchDescription([arm_driver])

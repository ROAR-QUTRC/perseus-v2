from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rmd_driver = Node(
        package="perseus_payloads",
        executable="rmd_driver",
        output="both",
    )
    rsbl_driver = Node(
        package="perseus_payloads",
        executable="rsbl_driver",
        output="both",
    )
    arm_controller = Node(
        package="perseus_payloads",
        executable="arm_controller",
        output="both",
    )

    return LaunchDescription([rmd_driver, rsbl_driver, arm_controller])
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

bt_pub_params = os.path.join(
    get_package_share_directory("autonomy"),
    "config",
    "bt_xml_publisher.yaml",
)

bt_xml_pub_node = Node(
    package="autonomy",
    executable="bt_xml_publisher",
    name="bt_xml_publisher",
    output="screen",
    parameters=[bt_pub_params],
)


def generate_launch_description():
    return LaunchDescription(
        [
            bt_xml_pub_node,
        ]
    )

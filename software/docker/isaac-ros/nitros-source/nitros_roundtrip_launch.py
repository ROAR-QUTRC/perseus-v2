"""
Stage 3 minimal NITROS round-trip test (see docs/source/systems/software/
isaac-ros-nitros-source-build.md). Replicates the composable-node setup from
NVIDIA's own isaac_ros_nitros_test_pol.py proof-of-life test, minus the
isaac_ros_test/launch_testing harness (which pulls in torch for an unrelated
model-mocking helper we don't need for this).

Two chained NitrosEmptyForwardNode composable nodes in one component
container, exercising real NITROS negotiation/type-adapter machinery:
  /topic_forward_input -> [nitros] -> /topic_forward_output
                        -> [nitros] -> /final/topic_forward_output

Not NVIDIA source -- our own minimal launch file.
"""
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='nitros_roundtrip_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_nitros',
                plugin='nvidia::isaac_ros::nitros::NitrosEmptyForwardNode',
                name='nitros_stage1',
                parameters=[{'compatible_format': 'nitros_empty'}],
            ),
            ComposableNode(
                package='isaac_ros_nitros',
                plugin='nvidia::isaac_ros::nitros::NitrosEmptyForwardNode',
                name='nitros_stage2',
                namespace='mid1',
                parameters=[{'compatible_format': 'nitros_empty'}],
                remappings=[
                    ('/mid1/topic_forward_input', '/topic_forward_output'),
                    ('/mid1/topic_forward_output', '/final/topic_forward_output'),
                ],
            ),
        ],
        output='both',
        arguments=['--ros-args', '--log-level', 'info'],
    )
    return LaunchDescription([container])

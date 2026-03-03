from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name="imu_bias_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",  # mt is nice for multiple callbacks
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="perseus_sensors",
                plugin="imu_processors::BiasEstimator",
                name="imu_bias_estimator",
                parameters=[
                    {
                        "use_odom": True,
                        "use_cmd_vel": False,
                        "accumulator_alpha": 0.01,
                        "stationary_mode": "AND",  # OR / AND
                        "imu_in_topic": "/livox/imu",
                        "odom_topic": "/odom",
                        "bias_out_topic": "/livox/gyro_bias",
                        "estimator_rate_hz": 100.0,
                    }
                ],
            ),
            ComposableNode(
                package="perseus_sensors",
                plugin="imu_processors::BiasRemover",
                name="imu_bias_remover",
                parameters=[
                    {
                        "imu_in_topic": "/livox/imu",
                        "bias_in_topic": "/livox/gyro_bias",
                        "imu_out_topic": "/livox/imu/corrected",
                        "output_rate_hz": 100.0,
                    }
                ],
            ),
        ],
    )

    return LaunchDescription([container])

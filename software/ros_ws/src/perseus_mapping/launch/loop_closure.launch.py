from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("perseus_mapping"),
        "config",
    )

    launch_config_file = os.path.join(config_path, "launch_config.yaml")
    with open(launch_config_file, "r") as f:
        launch_config = yaml.safe_load(f)

    use_sim_time = launch_config.get("use_sim_time", False)

    bridge_node = Node(
        package="perseus_mapping",
        executable="fastlio_maparray_bridge",
        name="fastlio_maparray_bridge",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "trans_for_mapupdate": 0.5,
            }
        ],
        output="screen",
    )

    graph_slam_node = Node(
        package="graph_based_slam",
        executable="graph_based_slam_node",
        name="graph_based_slam",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "registration_method": "GICP",
                "voxel_leaf_size": 0.1,
                "loop_detection_period": 1000,
                "threshold_loop_closure_score": 1.0,
                "distance_loop_closure": 15.0,
                "range_of_searching_loop_closure": 15.0,
                "search_submap_num": 2,
                "num_adjacent_pose_cnstraints": 5,
                "use_save_map_in_loop": False,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            bridge_node,
            graph_slam_node,
        ]
    )

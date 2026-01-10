# Copyright (c) 2025 Leander Stephen Desouza
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node, PushROSNamespace, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory('autonomy')

    namespace = LaunchConfiguration('namespace')
    keepout_mask_yaml_file = LaunchConfiguration('keepout_mask')
    keepout_mask_topic = LaunchConfiguration('keepout_mask_topic')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    use_intra_process_comms = LaunchConfiguration('use_intra_process_comms')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)

    use_respawn = LaunchConfiguration('use_respawn')
    use_keepout_zones = LaunchConfiguration('use_keepout_zones')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = [
        'keepout_filter_mask_server',
        'keepout_costmap_filter_info_server',
    ]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    yaml_substitutions = {
        'KEEPOUT_ZONE_ENABLED': use_keepout_zones,
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # -----------------------
    # Launch arguments
    # -----------------------
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_keepout_mask_yaml_cmd = DeclareLaunchArgument(
        'keepout_mask',
        default_value=os.path.join(bringup_dir, 'config', 'keepout_mask.yaml'),
        description='Full path to keepout mask yaml file to load',
    )

    # IMPORTANT: by default nav2_map_server publishes OccupancyGrid on "map".
    # This lets you publish the keepout mask on a dedicated topic instead,
    # so it won’t collide with SLAM Toolbox’s /map.
    declare_keepout_mask_topic_cmd = DeclareLaunchArgument(
        'keepout_mask_topic',
        default_value='keepout_filter_mask',
        description='Topic name for the keepout mask OccupancyGrid',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the lifecycle nodes',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'perseus_nav_params.yaml'),
        description='Full path to the ROS2 parameters file to use',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True',
    )

    declare_use_intra_process_comms_cmd = DeclareLaunchArgument(
        'use_intra_process_comms',
        default_value='False',
        description='Whether to use intra process communication',
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='Name of container that nodes load in if composition is used',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Respawn if a node crashes (only when composition is disabled)',
    )

    declare_use_keepout_zones_cmd = DeclareLaunchArgument(
        'use_keepout_zones',
        default_value='True',
        description='Enable keepout zones',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    # -----------------------
    # Non-composed nodes
    # -----------------------
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            PushROSNamespace(namespace),
            SetParameter('use_sim_time', use_sim_time),

            Node(
                condition=IfCondition(use_keepout_zones),
                package='nav2_map_server',
                executable='map_server',
                name='keepout_filter_mask_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    configured_params,
                    {'yaml_filename': keepout_mask_yaml_file},
                    {'topic_name': keepout_mask_topic},
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            Node(
                condition=IfCondition(use_keepout_zones),
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='keepout_costmap_filter_info_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_keepout_zone',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes},
                ],
            ),
        ],
    )

    # -----------------------
    # Composed nodes
    # -----------------------
    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            PushROSNamespace(namespace),
            SetParameter('use_sim_time', use_sim_time),

            LoadComposableNodes(
                target_container=container_name_full,
                condition=IfCondition(use_keepout_zones),
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='keepout_filter_mask_server',
                        parameters=[
                            configured_params,
                            {'yaml_filename': keepout_mask_yaml_file},
                            {'topic_name': keepout_mask_topic},
                        ],
                        remappings=remappings,
                        extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    ),
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::CostmapFilterInfoServer',
                        name='keepout_costmap_filter_info_server',
                        parameters=[configured_params],
                        remappings=remappings,
                        extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    ),
                ],
            ),

            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_keepout_zone',
                        parameters=[
                            {'autostart': autostart, 'node_names': lifecycle_nodes}
                        ],
                        extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    ),
                ],
            ),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_keepout_mask_yaml_cmd)
    ld.add_action(declare_keepout_mask_topic_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_intra_process_comms_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_use_keepout_zones_cmd)
    ld.add_action(declare_log_level_cmd)

    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld

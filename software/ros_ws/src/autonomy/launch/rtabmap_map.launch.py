import os

from launch import LaunchDescription, Substitution, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from typing import Text
from ament_index_python.packages import get_package_share_directory


class ConditionalText(Substitution):
    def __init__(self, text_if, text_else, condition):
        self.text_if = text_if
        self.text_else = text_else
        self.condition = condition

    def perform(self, context: 'LaunchContext') -> Text:
        if self.condition == True or self.condition == 'true' or self.condition == 'True':
            return self.text_if
        else:
            return self.text_else


class ConditionalBool(Substitution):
    def __init__(self, text_if, text_else, condition):
        self.text_if = text_if
        self.text_else = text_else
        self.condition = condition

    def perform(self, context: 'LaunchContext') -> bool:
        if self.condition:
            return self.text_if
        else:
            return self.text_else


def launch_setup(context, *args, **kwargs):

    rtabmap_viz_odometry_node_name = "rgbd_odometry"
    use_icp_odometry = LaunchConfiguration('icp_odometry').perform(context)
    use_icp_odometry = use_icp_odometry == 'true' or use_icp_odometry == 'True'
    use_stereo_odometry = LaunchConfiguration('stereo').perform(context)
    use_stereo_odometry = use_stereo_odometry == 'true' or use_stereo_odometry == 'True'
    if use_icp_odometry:
        rtabmap_viz_odometry_node_name = "icp_odometry"
    elif use_stereo_odometry:
        rtabmap_viz_odometry_node_name = "stereo_odometry"

    return [
        DeclareLaunchArgument('depth', default_value=ConditionalText('false', 'true', IfCondition(PythonExpression(["'", LaunchConfiguration('stereo'), "' == 'true'"]))._predicate_func(context)), description=''),
        DeclareLaunchArgument('subscribe_rgb', default_value=LaunchConfiguration('depth'), description=''),
        DeclareLaunchArgument('args',  default_value=LaunchConfiguration('rtabmap_args'), description='Can be used to pass RTAB-Map\'s parameters or other flags like --udebug and --delete_db_on_start/-d'),
        DeclareLaunchArgument('sync_queue_size',  default_value=LaunchConfiguration('queue_size'), description='Queue size of topic synchronizers.'),
        DeclareLaunchArgument('qos_image',       default_value=LaunchConfiguration('qos'), description='Specific QoS used for image input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_camera_info', default_value=LaunchConfiguration('qos'), description='Specific QoS used for camera info input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_scan',        default_value=LaunchConfiguration('qos'), description='Specific QoS used for scan input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_odom',        default_value=LaunchConfiguration('qos'), description='Specific QoS used for odometry input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_user_data',   default_value=LaunchConfiguration('qos'), description='Specific QoS used for user input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_imu',         default_value=LaunchConfiguration('qos'), description='Specific QoS used for imu input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_gps',         default_value=LaunchConfiguration('qos'), description='Specific QoS used for gps input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_env_sensor',  default_value=LaunchConfiguration('qos'), description='Specific QoS used for env sensor input data: 0=system default, 1=Reliable, 2=Best Effort.'),

        # These arguments should not be modified directly
        DeclareLaunchArgument('rgb_topic_relay',         default_value=ConditionalText(''.join([LaunchConfiguration('rgb_topic').perform(context), "_relay"]), ''.join(LaunchConfiguration('rgb_topic').perform(context)), LaunchConfiguration('compressed').perform(context)), description='Should not be modified manually!'),
        DeclareLaunchArgument('depth_topic_relay',       default_value=ConditionalText(''.join([LaunchConfiguration('depth_topic').perform(context), "_relay"]), ''.join(LaunchConfiguration('depth_topic').perform(context)), LaunchConfiguration('compressed').perform(context)), description='Should not be modified manually!'),
        DeclareLaunchArgument('left_image_topic_relay',  default_value=ConditionalText(''.join([LaunchConfiguration('left_image_topic').perform(context), "_relay"]), ''.join(LaunchConfiguration('left_image_topic').perform(context)), LaunchConfiguration('compressed').perform(context)), description='Should not be modified manually!'),
        DeclareLaunchArgument('right_image_topic_relay', default_value=ConditionalText(''.join([LaunchConfiguration('right_image_topic').perform(context), "_relay"]), ''.join(LaunchConfiguration('right_image_topic').perform(context)), LaunchConfiguration('compressed').perform(context)), description='Should not be modified manually!'),
        DeclareLaunchArgument('rgbd_topic_relay',        default_value=ConditionalText(''.join(LaunchConfiguration('rgbd_topic').perform(context)), ''.join([LaunchConfiguration('rgbd_topic').perform(context), "_relay"]), LaunchConfiguration('rgbd_sync').perform(context)), description='Should not be modified manually!'),

        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        # 'use_sim_time' will be set on all nodes following the line above

        # RTAB-Map SLAM node
        Node(
            package='rtabmap_slam', executable='rtabmap', name="rtabmap", output="screen",
            emulate_tty=True,
            parameters=[{
                "subscribe_depth": LaunchConfiguration('depth'),
                "subscribe_rgbd": LaunchConfiguration('subscribe_rgbd'),
                "subscribe_rgb": LaunchConfiguration('subscribe_rgb'),
                "subscribe_stereo": LaunchConfiguration('stereo'),
                "subscribe_scan": LaunchConfiguration('subscribe_scan'),
                "subscribe_scan_cloud": LaunchConfiguration('subscribe_scan_cloud'),
                "subscribe_user_data": LaunchConfiguration('subscribe_user_data'),
                "subscribe_odom_info": ConditionalBool(True, False, IfCondition(PythonExpression(["'", LaunchConfiguration('icp_odometry'), "' == 'true' or '", LaunchConfiguration('visual_odometry'), "' == 'true'"]))._predicate_func(context)).perform(context),
                "frame_id": LaunchConfiguration('frame_id'),
                "map_frame_id": LaunchConfiguration('map_frame_id'),
                "odom_frame_id": LaunchConfiguration('odom_frame_id').perform(context),
                "publish_tf": LaunchConfiguration('publish_tf_map'),
                "initial_pose": LaunchConfiguration('initial_pose'),
                "use_action_for_goal": LaunchConfiguration('use_action_for_goal'),
                "ground_truth_frame_id": LaunchConfiguration('ground_truth_frame_id').perform(context),
                "ground_truth_base_frame_id": LaunchConfiguration('ground_truth_base_frame_id').perform(context),
                "odom_tf_angular_variance": LaunchConfiguration('odom_tf_angular_variance'),
                "odom_tf_linear_variance": LaunchConfiguration('odom_tf_linear_variance'),
                "odom_sensor_sync": LaunchConfiguration('odom_sensor_sync'),
                "wait_for_transform": LaunchConfiguration('wait_for_transform'),
                "database_path": LaunchConfiguration('database_path'),
                "approx_sync": LaunchConfiguration('approx_sync'),
                "config_path": LaunchConfiguration('cfg').perform(context),
                "topic_queue_size": LaunchConfiguration('topic_queue_size'),
                "sync_queue_size": LaunchConfiguration('sync_queue_size'),
                "qos_image": LaunchConfiguration('qos_image'),
                "qos_scan": LaunchConfiguration('qos_scan'),
                "qos_odom": LaunchConfiguration('qos_odom'),
                "qos_camera_info": LaunchConfiguration('qos_camera_info'),
                "qos_imu": LaunchConfiguration('qos_imu'),
                "qos_gps": LaunchConfiguration('qos_gps'),
                "qos_env_sensor": LaunchConfiguration('qos_env_sensor'),
                "qos_user_data": LaunchConfiguration('qos_user_data'),
                "scan_normal_k": LaunchConfiguration('scan_normal_k'),
                "landmark_linear_variance": LaunchConfiguration('tag_linear_variance'),
                "landmark_angular_variance": LaunchConfiguration('tag_angular_variance'),
                "Mem/IncrementalMemory": ConditionalText("true", "false", IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' != 'true'"]))._predicate_func(context)).perform(context),
                "Mem/InitWMWithAllNodes": ConditionalText("true", "false", IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' == 'true'"]))._predicate_func(context)).perform(context),
                "Mem/SaveDepth16Format": "true",
                "Mem/DepthCompressionFormat": ".png",
                        # ADD THESE:
                "Grid/CellSize": "0.2",
                "Grid/RangeMax": "10.0",
                "Grid/RangeMin": "0.1",
                "RGBD/DepthMax": "10.0",
                "RGBD/DepthMin": "0.1",
            }],
            remappings=[
                ("map", LaunchConfiguration('map_topic')),
                ("rgb/image", LaunchConfiguration('rgb_topic_relay')),
                ("depth/image", LaunchConfiguration('depth_topic_relay')),
                ("rgb/camera_info", LaunchConfiguration('camera_info_topic')),
                ("rgbd_image", LaunchConfiguration('rgbd_topic_relay')),
                ("left/image_rect", LaunchConfiguration('left_image_topic_relay')),
                ("right/image_rect", LaunchConfiguration('right_image_topic_relay')),
                ("left/camera_info", LaunchConfiguration('left_camera_info_topic')),
                ("right/camera_info", LaunchConfiguration('right_camera_info_topic')),
                ("scan", LaunchConfiguration('scan_topic')),
                ("scan_cloud", LaunchConfiguration('scan_cloud_topic')),
                ("user_data", LaunchConfiguration('user_data_topic')),
                ("user_data_async", LaunchConfiguration('user_data_async_topic')),
                ("gps/fix", LaunchConfiguration('gps_topic')),
                ("tag_detections", LaunchConfiguration('tag_topic')),
                ("fiducial_transforms", LaunchConfiguration('fiducial_topic')),
                ("env_sensor", LaunchConfiguration('env_sensor_topic')),
                ("odom", LaunchConfiguration('odom_topic')),
                ("imu", LaunchConfiguration('imu_topic')),
                ("goal_out", LaunchConfiguration('output_goal_topic'))],
            arguments=[LaunchConfiguration("args"), "--ros-args", "--log-level", [LaunchConfiguration('namespace'), '.rtabmap:=', LaunchConfiguration('log_level')], "--log-level", ['rtabmap:=', LaunchConfiguration('log_level')]],
            prefix=LaunchConfiguration('launch_prefix'),
            namespace=LaunchConfiguration('namespace')),

        # # RTAB-Map Viz node (optional)
        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', name="rtabmap_viz", output='screen',
        #     emulate_tty=True,
        #     parameters=[{
        #         "subscribe_depth": LaunchConfiguration('depth'),
        #         "subscribe_rgbd": LaunchConfiguration('subscribe_rgbd'),
        #         "subscribe_rgb": LaunchConfiguration('subscribe_rgb'),
        #         "subscribe_stereo": LaunchConfiguration('stereo'),
        #         "subscribe_scan": LaunchConfiguration('subscribe_scan'),
        #         "subscribe_scan_cloud": LaunchConfiguration('subscribe_scan_cloud'),
        #         "subscribe_user_data": LaunchConfiguration('subscribe_user_data'),
        #         "subscribe_odom_info": ConditionalBool(True, False, IfCondition(PythonExpression(["'", LaunchConfiguration('icp_odometry'), "' == 'true' or '", LaunchConfiguration('visual_odometry'), "' == 'true'"]))._predicate_func(context)).perform(context),
        #         "frame_id": LaunchConfiguration('frame_id'),
        #         "odom_frame_id": LaunchConfiguration('odom_frame_id').perform(context),
        #         "wait_for_transform": LaunchConfiguration('wait_for_transform'),
        #         "approx_sync": LaunchConfiguration('approx_sync'),
        #         "topic_queue_size": LaunchConfiguration('topic_queue_size'),
        #         "sync_queue_size": LaunchConfiguration('sync_queue_size'),
        #         "qos_image": LaunchConfiguration('qos_image'),
        #         "qos_scan": LaunchConfiguration('qos_scan'),
        #         "qos_odom": LaunchConfiguration('qos_odom'),
        #         "qos_camera_info": LaunchConfiguration('qos_camera_info'),
        #         "qos_user_data": LaunchConfiguration('qos_user_data'),
        #         "odometry_node_name": rtabmap_viz_odometry_node_name
        #     }],
        #     remappings=[
        #         ("rgb/image", LaunchConfiguration('rgb_topic_relay')),
        #         ("depth/image", LaunchConfiguration('depth_topic_relay')),
        #         ("rgb/camera_info", LaunchConfiguration('camera_info_topic')),
        #         ("rgbd_image", LaunchConfiguration('rgbd_topic_relay')),
        #         ("left/image_rect", LaunchConfiguration('left_image_topic_relay')),
        #         ("right/image_rect", LaunchConfiguration('right_image_topic_relay')),
        #         ("left/camera_info", LaunchConfiguration('left_camera_info_topic')),
        #         ("right/camera_info", LaunchConfiguration('right_camera_info_topic')),
        #         ("scan", LaunchConfiguration('scan_topic')),
        #         ("scan_cloud", LaunchConfiguration('scan_cloud_topic')),
        #         ("odom", LaunchConfiguration('odom_topic'))],
        #     condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
        #     arguments=[LaunchConfiguration("gui_cfg"), "--ros-args", "--log-level", [LaunchConfiguration('namespace'), '.rtabmap_viz:=', LaunchConfiguration('log_level')], "--log-level", ['rtabmap_viz:=', LaunchConfiguration('log_level')]],
        #     prefix=LaunchConfiguration('launch_prefix'),
        #     namespace=LaunchConfiguration('namespace')),

        # Point cloud for RViz (optional)
        Node(
            package='rtabmap_util', executable='point_cloud_xyzrgb', name="point_cloud_xyzrgb", output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{
                "decimation": 4,
                "voxel_size": 0.0,
                "approx_sync": LaunchConfiguration('approx_sync'),
                "approx_sync_max_interval": LaunchConfiguration('approx_sync_max_interval'),
                "qos": LaunchConfiguration('qos_image'),
                "qos_camera_info": LaunchConfiguration('qos_camera_info')
            }],
            remappings=[
                ('left/image', LaunchConfiguration('left_image_topic_relay')),
                ('right/image', LaunchConfiguration('right_image_topic_relay')),
                ('left/camera_info', LaunchConfiguration('left_camera_info_topic')),
                ('right/camera_info', LaunchConfiguration('right_camera_info_topic')),
                ('rgb/image', LaunchConfiguration('rgb_topic_relay')),
                ('depth/image', LaunchConfiguration('depth_topic_relay')),
                ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
                ('rgbd_image', LaunchConfiguration('rgbd_topic_relay')),
                ('cloud', 'voxel_cloud')]),
    ]


def generate_launch_description():

    return LaunchDescription([

        # Mode
        DeclareLaunchArgument('stereo',       default_value='false', description='Use stereo input instead of RGB-D.'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode (true) or mapping mode (false).'),
        DeclareLaunchArgument('rtabmap_viz',  default_value='true',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='false', description='Launch RViz point cloud visualization (optional).'),

        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true.'),

        DeclareLaunchArgument('log_level',    default_value='info',  description='ROS logging level (debug, info, warn, error).'),

        # Config files
        DeclareLaunchArgument('cfg',     default_value='',                       description='Path to RTAB-Map config file (*.ini).'),
        DeclareLaunchArgument('gui_cfg', default_value='~/.ros/rtabmap_gui.ini', description='Configuration path of rtabmap_viz.'),

        # Frames
        DeclareLaunchArgument('frame_id',       default_value='base_link',         description='Robot base frame id.'),
        DeclareLaunchArgument('odom_frame_id',  default_value='odom',              description='Odometry frame id. If set, TF is used to get odometry instead of the topic.'),
        DeclareLaunchArgument('map_frame_id',   default_value='map',               description='Output map frame id (TF).'),
        DeclareLaunchArgument('map_topic',      default_value='map_3d',            description='Map topic name.'),
        DeclareLaunchArgument('publish_tf_map', default_value='true',              description='Publish TF between map and odometry frames.'),

        # General
        DeclareLaunchArgument('namespace',        default_value='rtabmap',           description='Namespace for RTAB-Map nodes.'),
        DeclareLaunchArgument('database_path',    default_value='~/.ros/rtabmap.db', description='Where the map is saved/loaded.'),
        DeclareLaunchArgument('topic_queue_size', default_value='10',                description='Queue size of individual topic subscribers.'),
        DeclareLaunchArgument('queue_size',       default_value='10',                description='Backward compatibility, use "sync_queue_size" instead.'),
        DeclareLaunchArgument('qos',              default_value='0',                 description='General QoS: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('wait_for_transform', default_value='0.2',             description='Wait for transform timeout (seconds).'),
        DeclareLaunchArgument('rtabmap_args',     default_value='',                  description='Extra RTAB-Map arguments (e.g., --delete_db_on_start).'),
        DeclareLaunchArgument('launch_prefix',    default_value='',                  description='Node launch prefix, e.g., "xterm -e gdb -ex run --args".'),
        DeclareLaunchArgument('output',           default_value='screen',            description='Control node output (screen or log).'),
        DeclareLaunchArgument('initial_pose',     default_value='',                  description='Initial pose in localization mode: "x y z roll pitch yaw".'),

        DeclareLaunchArgument('output_goal_topic',    default_value='/goal_pose', description='Output goal topic (can be connected to nav2).'),
        DeclareLaunchArgument('use_action_for_goal',  default_value='false',      description='Connect to nav2\'s navigate_to_pose action server.'),

        # Ground truth (optional)
        DeclareLaunchArgument('ground_truth_frame_id',      default_value='', description='e.g., "world"'),
        DeclareLaunchArgument('ground_truth_base_frame_id', default_value='', description='e.g., "tracker"'),

        # Synchronization
        DeclareLaunchArgument('approx_sync',              default_value='true', description='Use approximate time synchronization policy.'),
        DeclareLaunchArgument('approx_sync_max_interval', default_value='0.0',  description='Max interval (sec) for approx_sync. 0 = infinite.'),

        # RGB-D topics
        DeclareLaunchArgument('rgb_topic',         default_value='/camera/camera/color/image_raw',                    description='RGB image topic.'),
        DeclareLaunchArgument('depth_topic',       default_value='/camera/camera/aligned_depth_to_color/image_raw',   description='Depth image topic.'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/color/camera_info',                  description='Camera info topic.'),

        # Stereo topics
        DeclareLaunchArgument('stereo_namespace',        default_value='/stereo_camera',                                               description='Stereo camera namespace.'),
        DeclareLaunchArgument('left_image_topic',        default_value=[LaunchConfiguration('stereo_namespace'), '/left/image_rect_color'], description='Left stereo image topic.'),
        DeclareLaunchArgument('right_image_topic',       default_value=[LaunchConfiguration('stereo_namespace'), '/right/image_rect'],      description='Right stereo image topic.'),
        DeclareLaunchArgument('left_camera_info_topic',  default_value=[LaunchConfiguration('stereo_namespace'), '/left/camera_info'],      description='Left camera info topic.'),
        DeclareLaunchArgument('right_camera_info_topic', default_value=[LaunchConfiguration('stereo_namespace'), '/right/camera_info'],     description='Right camera info topic.'),

        # RGBD sync
        DeclareLaunchArgument('rgbd_sync',        default_value='true',                       description='Pre-sync rgb/depth/camera_info topics.'),
        DeclareLaunchArgument('approx_rgbd_sync', default_value='true',                       description='Use approximate sync for RGBD. false=exact.'),
        DeclareLaunchArgument('subscribe_rgbd',   default_value=LaunchConfiguration('rgbd_sync'), description='Subscribe to pre-synced RGBDImage topic.'),
        DeclareLaunchArgument('rgbd_topic',       default_value='rgbd_image',                 description='Pre-synced RGBD topic name.'),
        # 1 is mm, 1000 is m, etc.
        DeclareLaunchArgument('depth_scale',      default_value='1.0',                        description='Depth image scale factor.'),

        # Image compression
        DeclareLaunchArgument('compressed',            default_value='false',          description='Subscribe to compressed image topics.'),
        DeclareLaunchArgument('rgb_image_transport',   default_value='compressed',     description='RGB image transport type.'),
        DeclareLaunchArgument('depth_image_transport', default_value='compressedDepth', description='Depth image transport type.'),

        # LiDAR
        DeclareLaunchArgument('subscribe_scan',       default_value='false',       description='Subscribe to laser scan topic.'),
        DeclareLaunchArgument('scan_topic',           default_value='/scan',       description='Laser scan topic name.'),
        DeclareLaunchArgument('subscribe_scan_cloud', default_value='false',       description='Subscribe to point cloud scan topic.'),
        DeclareLaunchArgument('scan_cloud_topic',     default_value='/scan_cloud', description='Point cloud scan topic name.'),
        DeclareLaunchArgument('scan_normal_k',        default_value='0',           description='Number of neighbors for scan normal estimation.'),

        # Odometry (consumed by SLAM node, not launched here)
        DeclareLaunchArgument('visual_odometry',          default_value='false',             description='Whether an external visual odometry node is running (for subscribe_odom_info).'),
        DeclareLaunchArgument('icp_odometry',             default_value='false',             description='Whether an external ICP odometry node is running (for subscribe_odom_info).'),
        DeclareLaunchArgument('odom_topic',               default_value='/odometry/filtered', description='Odometry topic name.'),
        DeclareLaunchArgument('odom_tf_angular_variance', default_value='0.01',              description='Default angular variance when TF is used for odometry.'),
        DeclareLaunchArgument('odom_tf_linear_variance',  default_value='0.001',             description='Default linear variance when TF is used for odometry.'),
        DeclareLaunchArgument('odom_sensor_sync',         default_value='false',             description='Enable odometry sensor sync.'),

        # IMU
        DeclareLaunchArgument('imu_topic',        default_value='/imu/data', description='IMU topic for gravity constraints.'),

        # User data
        DeclareLaunchArgument('subscribe_user_data',   default_value='false',            description='Subscribe to user data topic (synchronized).'),
        DeclareLaunchArgument('user_data_topic',       default_value='/user_data',       description='User data topic name.'),
        DeclareLaunchArgument('user_data_async_topic', default_value='/user_data_async', description='User data async topic name.'),

        # GPS
        DeclareLaunchArgument('gps_topic', default_value='/gps/fix', description='GPS topic for graph optimization.'),

        # Tags / Landmarks
        DeclareLaunchArgument('tag_topic',            default_value='/detections',          description='AprilTag detections topic.'),
        DeclareLaunchArgument('tag_linear_variance',  default_value='0.0001',               description='Linear variance for tag landmarks.'),
        DeclareLaunchArgument('tag_angular_variance', default_value='9999.0',               description='Angular variance for tag landmarks (>=9999 ignores rotation).'),
        DeclareLaunchArgument('fiducial_topic',       default_value='/fiducial_transforms', description='aruco_detect fiducial transforms topic.'),

        # Environment sensor
        DeclareLaunchArgument('env_sensor_topic', default_value='/env_sensor', description='rtabmap_msgs/EnvSensor topic.'),

        OpaqueFunction(function=launch_setup)
    ])
from launch import LaunchDescription, Substitution, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from typing import Text

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
            
def launch_setup(context, *args, **kwargs):      

    return [
        DeclareLaunchArgument('depth', default_value=ConditionalText('false', 'true', IfCondition(PythonExpression(["'", LaunchConfiguration('stereo'), "' == 'true'"]))._predicate_func(context)), description=''),
        DeclareLaunchArgument('subscribe_rgb', default_value=LaunchConfiguration('depth'), description=''),
        DeclareLaunchArgument('rtabmap_args',   default_value='--delete_db_on_start',                   description='Backward compatibility, use "args" instead.'),
        DeclareLaunchArgument('args',  default_value=LaunchConfiguration('rtabmap_args'), description='Can be used to pass RTAB-Map\'s parameters or other flags like --udebug and --delete_db_on_start/-d'),
        DeclareLaunchArgument('sync_queue_size',  default_value=LaunchConfiguration('queue_size'), description='Queue size of topic synchronizers.'),
        DeclareLaunchArgument('qos_image',       default_value=LaunchConfiguration('qos'), description='Specific QoS used for image input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_camera_info', default_value=LaunchConfiguration('qos'), description='Specific QoS used for camera info input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_scan',        default_value=LaunchConfiguration('qos'), description='Specific QoS used for scan input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_odom',        default_value=LaunchConfiguration('qos'), description='Specific QoS used for odometry input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_user_data',   default_value=LaunchConfiguration('qos'), description='Specific QoS used for user input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_imu',         default_value=LaunchConfiguration('qos'), description='Specific QoS used for imu input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_gps',         default_value=LaunchConfiguration('qos'), description='Specific QoS used for gps input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_env_sensor',         default_value=LaunchConfiguration('qos'), description='Specific QoS used for env sensor input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('odom_log_level',  default_value=LaunchConfiguration('log_level'), description='Specific ROS logger level for odometry node.'),
        
        #These arguments should not be modified directly, see referred topics without "_relay" suffix above
        DeclareLaunchArgument('rgb_topic_relay',      default_value=ConditionalText(''.join([LaunchConfiguration('rgb_topic').perform(context), "_relay"]), ''.join(LaunchConfiguration('rgb_topic').perform(context)), LaunchConfiguration('compressed').perform(context)), description='Should not be modified manually!'),
        DeclareLaunchArgument('depth_topic_relay',      default_value=ConditionalText(''.join([LaunchConfiguration('depth_topic').perform(context), "_relay"]), ''.join(LaunchConfiguration('depth_topic').perform(context)), LaunchConfiguration('compressed').perform(context)), description='Should not be modified manually!'),
        DeclareLaunchArgument('left_image_topic_relay',      default_value=ConditionalText(''.join([LaunchConfiguration('left_image_topic').perform(context), "_relay"]), ''.join(LaunchConfiguration('left_image_topic').perform(context)), LaunchConfiguration('compressed').perform(context)), description='Should not be modified manually!'),
        DeclareLaunchArgument('right_image_topic_relay',      default_value=ConditionalText(''.join([LaunchConfiguration('right_image_topic').perform(context), "_relay"]), ''.join(LaunchConfiguration('right_image_topic').perform(context)), LaunchConfiguration('compressed').perform(context)), description='Should not be modified manually!'),
        DeclareLaunchArgument('rgbd_topic_relay',      default_value=ConditionalText(''.join(LaunchConfiguration('rgbd_topic').perform(context)), ''.join([LaunchConfiguration('rgbd_topic').perform(context), "_relay"]), LaunchConfiguration('rgbd_sync').perform(context)), description='Should not be modified manually!'),
    
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        # 'use_sim_time' will be set on all nodes following the line above
    
        # Relays RGB-Depth
        Node(
            package='image_transport', executable='republish', name='republish_rgb',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('stereo'), "' != 'true' and ('", LaunchConfiguration('subscribe_rgbd'), "' != 'true' or '", LaunchConfiguration('rgbd_sync'),"'=='true') and '", LaunchConfiguration('compressed'), "' == 'true'"])),
            remappings=[
                (['in/', LaunchConfiguration('rgb_image_transport')], [LaunchConfiguration('rgb_topic'), '/', LaunchConfiguration('rgb_image_transport')]),
                ('out', LaunchConfiguration('rgb_topic_relay'))], 
            arguments=[LaunchConfiguration('rgb_image_transport'), 'raw'],
            namespace=LaunchConfiguration('namespace')),
        Node(
            package='image_transport', executable='republish', name='republish_depth',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('stereo'), "' != 'true' and ('", LaunchConfiguration('subscribe_rgbd'), "' != 'true' or '", LaunchConfiguration('rgbd_sync'),"'=='true') and '", LaunchConfiguration('compressed'), "' == 'true'"])),
            remappings=[
                (['in/', LaunchConfiguration('depth_image_transport')], [LaunchConfiguration('depth_topic'), '/', LaunchConfiguration('depth_image_transport')]),
                ('out', LaunchConfiguration('depth_topic_relay'))], 
            arguments=[LaunchConfiguration('depth_image_transport'), 'raw'],
            namespace=LaunchConfiguration('namespace')),
        Node(
            package='rtabmap_sync', executable='rgbd_sync', name="rgbd_sync", output="screen",
            emulate_tty=True,
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('stereo'), "' != 'true' and '", LaunchConfiguration('rgbd_sync'), "' == 'true'"])),
            parameters=[{
                "approx_sync": LaunchConfiguration('approx_rgbd_sync'),
                "approx_sync_max_interval": LaunchConfiguration('approx_sync_max_interval'),
                "topic_queue_size": LaunchConfiguration('topic_queue_size'),
                "sync_queue_size": LaunchConfiguration('sync_queue_size'),
                "qos": LaunchConfiguration('qos_image'),
                "qos_camera_info": LaunchConfiguration('qos_camera_info'),
                "depth_scale": LaunchConfiguration('depth_scale')}],
            remappings=[
                ("rgb/image", LaunchConfiguration('rgb_topic_relay')),
                ("depth/image", LaunchConfiguration('depth_topic_relay')),
                ("rgb/camera_info", LaunchConfiguration('camera_info_topic')),
                ("rgbd_image", LaunchConfiguration('rgbd_topic_relay'))],
            namespace=LaunchConfiguration('namespace')),
            
        # Relays Stereo
        Node(
            package='image_transport', executable='republish', name='republish_left',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('stereo'), "' == 'true' and ('", LaunchConfiguration('subscribe_rgbd'), "' != 'true' or '", LaunchConfiguration('rgbd_sync'),"'=='true') and '", LaunchConfiguration('compressed'), "' == 'true'"])),
            remappings=[
                (['in/', LaunchConfiguration('rgb_image_transport')], [LaunchConfiguration('left_image_topic'), '/', LaunchConfiguration('rgb_image_transport')]),
                ('out', LaunchConfiguration('left_image_topic_relay'))], 
            arguments=[LaunchConfiguration('rgb_image_transport'), 'raw'],
            namespace=LaunchConfiguration('namespace')),
        Node(
            package='image_transport', executable='republish', name='republish_right',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('stereo'), "' == 'true' and ('", LaunchConfiguration('subscribe_rgbd'), "' != 'true' or '", LaunchConfiguration('rgbd_sync'),"'=='true') and '", LaunchConfiguration('compressed'), "' == 'true'"])),
            remappings=[
                (['in/', LaunchConfiguration('rgb_image_transport')], [LaunchConfiguration('right_image_topic'), '/', LaunchConfiguration('rgb_image_transport')]),
                ('out', LaunchConfiguration('right_image_topic_relay'))], 
            arguments=[LaunchConfiguration('rgb_image_transport'), 'raw'],
            namespace=LaunchConfiguration('namespace')),
        Node(
            package='rtabmap_sync', executable='stereo_sync', name="stereo_sync", output="screen",
            emulate_tty=True,
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('stereo'), "' == 'true' and '", LaunchConfiguration('rgbd_sync'), "' == 'true'"])),
            parameters=[{
                "approx_sync": LaunchConfiguration('approx_rgbd_sync'),
                "approx_sync_max_interval": LaunchConfiguration('approx_sync_max_interval'),
                "topic_queue_size": LaunchConfiguration('topic_queue_size'),
                "sync_queue_size": LaunchConfiguration('sync_queue_size'),
                "qos": LaunchConfiguration('qos_image'),
                "qos_camera_info": LaunchConfiguration('qos_camera_info')}],
            remappings=[
                ("left/image_rect", LaunchConfiguration('left_image_topic_relay')),
                ("right/image_rect", LaunchConfiguration('right_image_topic_relay')),
                ("left/camera_info", LaunchConfiguration('left_camera_info_topic')),
                ("right/camera_info", LaunchConfiguration('right_camera_info_topic')),
                ("rgbd_image", LaunchConfiguration('rgbd_topic_relay'))],
            namespace=LaunchConfiguration('namespace')),
            
        # Relay rgbd_image
        Node(
            package='rtabmap_util', executable='rgbd_relay', name="rgbd_relay", output="screen",
            emulate_tty=True,
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('rgbd_sync'), "' != 'true' and '", LaunchConfiguration('subscribe_rgbd'), "' == 'true' and '", LaunchConfiguration('compressed'), "' != 'true'"])),
            parameters=[{
                "qos": LaunchConfiguration('qos_image')}],
            remappings=[
                ("rgbd_image", LaunchConfiguration('rgbd_topic')),
                ("rgbd_image_relay", LaunchConfiguration('rgbd_topic_relay'))],
            namespace=LaunchConfiguration('namespace')),
        Node(
            package='rtabmap_util', executable='rgbd_relay', name="rgbd_relay_uncompress", output="screen",
            emulate_tty=True,
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('rgbd_sync'), "' != 'true' and '", LaunchConfiguration('subscribe_rgbd'), "' == 'true' and '", LaunchConfiguration('compressed'), "' == 'true'"])),
            parameters=[{
                "uncompress": True,
                "qos": LaunchConfiguration('qos_image')}],
            remappings=[
                ("rgbd_image", [LaunchConfiguration('rgbd_topic'), "/compressed"]),
                ("rgbd_image_relay", LaunchConfiguration('rgbd_topic_relay'))],
            namespace=LaunchConfiguration('namespace')),
                
        # RGB-D odometry
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', name="rgbd_odometry", output="screen",
            emulate_tty=True,
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('icp_odometry'), "' != 'true' and '", LaunchConfiguration('visual_odometry'), "' == 'true' and '", LaunchConfiguration('stereo'), "' != 'true'"])),
            parameters=[{
                "frame_id": LaunchConfiguration('frame_id'),
                "odom_frame_id": LaunchConfiguration('vo_frame_id'),
                "publish_tf": LaunchConfiguration('publish_tf_odom'),
                "ground_truth_frame_id": LaunchConfiguration('ground_truth_frame_id').perform(context),
                "ground_truth_base_frame_id": LaunchConfiguration('ground_truth_base_frame_id').perform(context),
                "wait_for_transform": LaunchConfiguration('wait_for_transform'),
                "wait_imu_to_init": LaunchConfiguration('wait_imu_to_init'),
                "always_check_imu_tf": LaunchConfiguration('always_check_imu_tf'),
                "approx_sync": LaunchConfiguration('approx_sync'),
                "approx_sync_max_interval": LaunchConfiguration('approx_sync_max_interval'),
                "config_path": LaunchConfiguration('cfg').perform(context),
                "topic_queue_size": LaunchConfiguration('topic_queue_size'),
                "sync_queue_size": LaunchConfiguration('sync_queue_size'),
                "qos": LaunchConfiguration('qos_image'),
                "qos_camera_info": LaunchConfiguration('qos_camera_info'),
                "qos_imu": LaunchConfiguration('qos_imu'),
                "subscribe_rgbd": LaunchConfiguration('subscribe_rgbd'),
                "guess_frame_id": LaunchConfiguration('odom_guess_frame_id').perform(context),
                "guess_min_translation": LaunchConfiguration('odom_guess_min_translation'),
                "guess_min_rotation": LaunchConfiguration('odom_guess_min_rotation'),
                "always_process_most_recent_frame": LaunchConfiguration('odom_always_process_most_recent_frame'),
                "rgbd_odometry_image_downscale": LaunchConfiguration('rgbd_odometry_image_downscale')}],
            remappings=[
                ("rgb/image", LaunchConfiguration('rgb_topic_relay')),
                ("depth/image", LaunchConfiguration('depth_topic_relay')),
                ("rgb/camera_info", LaunchConfiguration('camera_info_topic')),
                ("rgbd_image", LaunchConfiguration('rgbd_topic_relay')),
                ("odom", LaunchConfiguration('odom_topic')),
                ("imu", LaunchConfiguration('imu_topic'))],
            arguments=[LaunchConfiguration("args"), LaunchConfiguration("odom_args"), "--Vis/MinDepth", LaunchConfiguration('odom_min_depth'), "--ros-args", "--log-level", [LaunchConfiguration('namespace'), '.rgbd_odometry:=', LaunchConfiguration('odom_log_level')], "--log-level", ['rgbd_odometry:=', LaunchConfiguration('odom_log_level')]],
            prefix=LaunchConfiguration('launch_prefix'),
            namespace=LaunchConfiguration('namespace')),
        
        # Stereo odometry
        Node(
            package='rtabmap_odom', executable='stereo_odometry', name="stereo_odometry", output="screen",
            emulate_tty=True,
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('icp_odometry'), "' != 'true' and '", LaunchConfiguration('visual_odometry'), "' == 'true' and '", LaunchConfiguration('stereo'), "' == 'true'"])),
            parameters=[{
                "frame_id": LaunchConfiguration('frame_id'),
                "odom_frame_id": LaunchConfiguration('vo_frame_id'),
                "publish_tf": LaunchConfiguration('publish_tf_odom'),
                "ground_truth_frame_id": LaunchConfiguration('ground_truth_frame_id').perform(context),
                "ground_truth_base_frame_id": LaunchConfiguration('ground_truth_base_frame_id').perform(context),
                "wait_for_transform": LaunchConfiguration('wait_for_transform'),
                "wait_imu_to_init": LaunchConfiguration('wait_imu_to_init'),
                "always_check_imu_tf": LaunchConfiguration('always_check_imu_tf'),
                "approx_sync": LaunchConfiguration('approx_sync'),
                "approx_sync_max_interval": LaunchConfiguration('approx_sync_max_interval'),
                "config_path": LaunchConfiguration('cfg').perform(context),
                "topic_queue_size": LaunchConfiguration('topic_queue_size'),
                "sync_queue_size": LaunchConfiguration('sync_queue_size'),
                "qos": LaunchConfiguration('qos_image'),
                "qos_camera_info": LaunchConfiguration('qos_camera_info'),
                "qos_imu": LaunchConfiguration('qos_imu'),
                "subscribe_rgbd": LaunchConfiguration('subscribe_rgbd'),
                "guess_frame_id": LaunchConfiguration('odom_guess_frame_id').perform(context),
                "guess_min_translation": LaunchConfiguration('odom_guess_min_translation'),
                "guess_min_rotation": LaunchConfiguration('odom_guess_min_rotation'),
                "always_process_most_recent_frame": LaunchConfiguration('odom_always_process_most_recent_frame')}],
            remappings=[
                ("left/image_rect", LaunchConfiguration('left_image_topic_relay')),
                ("right/image_rect", LaunchConfiguration('right_image_topic_relay')),
                ("left/camera_info", LaunchConfiguration('left_camera_info_topic')),
                ("right/camera_info", LaunchConfiguration('right_camera_info_topic')),
                ("rgbd_image", LaunchConfiguration('rgbd_topic_relay')),
                ("odom", LaunchConfiguration('odom_topic')),
                ("imu", LaunchConfiguration('imu_topic'))],
            arguments=[LaunchConfiguration("args"), LaunchConfiguration("odom_args"), "--ros-args", "--log-level", [LaunchConfiguration('namespace'), '.stereo_odometry:=', LaunchConfiguration('odom_log_level')], "--log-level", ['stereo_odometry:=', LaunchConfiguration('odom_log_level')]],
            prefix=LaunchConfiguration('launch_prefix'),
            namespace=LaunchConfiguration('namespace')),
            
        # ICP odometry
        Node(
            package='rtabmap_odom', executable='icp_odometry', name="icp_odometry", output="screen",
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('icp_odometry')),
            parameters=[{
                "frame_id": LaunchConfiguration('frame_id'),
                "odom_frame_id": LaunchConfiguration('vo_frame_id'),
                "publish_tf": LaunchConfiguration('publish_tf_odom'),
                "ground_truth_frame_id": LaunchConfiguration('ground_truth_frame_id').perform(context),
                "ground_truth_base_frame_id": LaunchConfiguration('ground_truth_base_frame_id').perform(context),
                "wait_for_transform": LaunchConfiguration('wait_for_transform'),
                "wait_imu_to_init": LaunchConfiguration('wait_imu_to_init'),
                "always_check_imu_tf": LaunchConfiguration('always_check_imu_tf'),
                "approx_sync": LaunchConfiguration('approx_sync'),
                "config_path": LaunchConfiguration('cfg').perform(context),
                "topic_queue_size": LaunchConfiguration('topic_queue_size'),
                "sync_queue_size": LaunchConfiguration('sync_queue_size'),
                "qos": LaunchConfiguration('qos_image'),
                "qos_imu": LaunchConfiguration('qos_imu'),
                "guess_frame_id": LaunchConfiguration('odom_guess_frame_id').perform(context),
                "guess_min_translation": LaunchConfiguration('odom_guess_min_translation'),
                "guess_min_rotation": LaunchConfiguration('odom_guess_min_rotation'),
                "always_process_most_recent_frame": LaunchConfiguration('odom_always_process_most_recent_frame')}],
            remappings=[
                ("scan", LaunchConfiguration('scan_topic')),
                ("scan_cloud", LaunchConfiguration('scan_cloud_topic')),
                ("odom", LaunchConfiguration('odom_topic')),
                ("imu", LaunchConfiguration('imu_topic'))],
            arguments=[LaunchConfiguration("args"), LaunchConfiguration("odom_args"), "--ros-args", "--log-level", [LaunchConfiguration('namespace'), '.icp_odometry:=', LaunchConfiguration('odom_log_level')], "--log-level", ['icp_odometry:=', LaunchConfiguration('odom_log_level')]],
            prefix=LaunchConfiguration('launch_prefix'),
            namespace=LaunchConfiguration('namespace')),


        ]

def generate_launch_description():
    
    
    return LaunchDescription([
        
        # Arguments
        DeclareLaunchArgument('stereo', default_value='false', description='Use stereo input instead of RGB-D.'),

        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument('log_level',    default_value='info', description="ROS logging level (debug, info, warn, error)."),

        # Config files
        DeclareLaunchArgument('cfg',      default_value='',                        description='Path to odometry config file (*.ini).'),

        DeclareLaunchArgument('frame_id',       default_value='base_link',          description='Fixed frame id of the robot (base frame).'),
        DeclareLaunchArgument('odom_frame_id',  default_value='',                   description='If set, TF is used to get odometry instead of the topic.'),
        DeclareLaunchArgument('namespace',      default_value='rtabmap',            description='Namespace for odometry nodes.'),
        DeclareLaunchArgument('topic_queue_size', default_value='5',               description='Queue size of individual topic subscribers.'),
        DeclareLaunchArgument('queue_size',     default_value='5',                 description='Backward compatibility, use "sync_queue_size" instead.'),
        DeclareLaunchArgument('qos',            default_value='0',                  description='General QoS used for sensor input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('wait_for_transform', default_value='0.2',            description='Wait for transform timeout.'),
        DeclareLaunchArgument('launch_prefix',  default_value='',                   description='For debugging purpose, it fills prefix tag of the nodes, e.g., "xterm -e gdb -ex run --args"'),
        DeclareLaunchArgument('output',         default_value='screen',             description='Control node output (screen or log).'),

        DeclareLaunchArgument('ground_truth_frame_id',      default_value='', description='e.g., "world"'),
        DeclareLaunchArgument('ground_truth_base_frame_id', default_value='', description='e.g., "tracker"'),
        
        DeclareLaunchArgument('approx_sync',  default_value='true',            description='If timestamps of the input topics should be synchronized using approximate or exact time policy.'),
        DeclareLaunchArgument('approx_sync_max_interval',  default_value='0.04', description='(sec) 0 means infinite interval duration (used with approx_sync=true)'),

        # RGB-D related topics
        DeclareLaunchArgument('rgb_topic',           default_value='/camera/camera/color/image_raw',       description=''),
        DeclareLaunchArgument('depth_topic',         default_value='/camera/camera/depth/image_rect_raw', description=''),
        DeclareLaunchArgument('camera_info_topic',   default_value='/camera/camera/color/camera_info',            description=''),
     
        # Stereo related topics
        DeclareLaunchArgument('stereo_namespace',        default_value='/stereo_camera', description=''),
        DeclareLaunchArgument('left_image_topic',        default_value=[LaunchConfiguration('stereo_namespace'), '/left/image_rect_color'], description=''),
        DeclareLaunchArgument('right_image_topic',       default_value=[LaunchConfiguration('stereo_namespace'), '/right/image_rect'], description='Use grayscale image for efficiency'),
        DeclareLaunchArgument('left_camera_info_topic',  default_value=[LaunchConfiguration('stereo_namespace'), '/left/camera_info'], description=''),
        DeclareLaunchArgument('right_camera_info_topic', default_value=[LaunchConfiguration('stereo_namespace'), '/right/camera_info'], description=''),
        
        # Use Pre-sync RGBDImage format
        DeclareLaunchArgument('rgbd_sync',        default_value='true',      description='Pre-sync rgb_topic, depth_topic, camera_info_topic.'),
        DeclareLaunchArgument('approx_rgbd_sync', default_value='true',       description='false=exact synchronization.'),
        DeclareLaunchArgument('subscribe_rgbd',   default_value=LaunchConfiguration('rgbd_sync'), description='Already synchronized RGB-D related topic.'),
        DeclareLaunchArgument('rgbd_topic',       default_value='rgbd_image', description=''),
        DeclareLaunchArgument('depth_scale',      default_value='1.0',        description=''),
        DeclareLaunchArgument('rgbd_odometry_image_downscale', default_value='4'),
        # Image topic compression
        DeclareLaunchArgument('compressed',            default_value='false', description='If you want to subscribe to compressed image topics'),
        DeclareLaunchArgument('rgb_image_transport',   default_value='compressed', description='Common types: compressed, theora'),
        DeclareLaunchArgument('depth_image_transport', default_value='compressedDepth', description='Depth compatible types: compressedDepth'),
       
        # LiDAR
        DeclareLaunchArgument('subscribe_scan',       default_value='false',       description='Subscribe to laser scan topic.'),
        DeclareLaunchArgument('scan_topic',           default_value='/scan',       description='Laser scan topic name.'),
        DeclareLaunchArgument('subscribe_scan_cloud', default_value='false',       description='Subscribe to point cloud topic.'),
        DeclareLaunchArgument('scan_cloud_topic',     default_value='/scan_cloud', description='Point cloud topic name.'),
        DeclareLaunchArgument('scan_normal_k',        default_value='0',           description=''),
        
        # Odometry
        DeclareLaunchArgument('visual_odometry',            default_value='true',  description='Launch rtabmap visual odometry node.'),
        DeclareLaunchArgument('icp_odometry',               default_value='false', description='Launch rtabmap icp odometry node.'),
        DeclareLaunchArgument('odom_topic',                 default_value='odom',  description='Odometry topic name.'),
        DeclareLaunchArgument('vo_frame_id',                default_value=LaunchConfiguration('odom_topic'), description='Visual/Icp odometry frame ID for TF.'),
        DeclareLaunchArgument('publish_tf_odom',            default_value='true',  description='Publish TF frames for odometry.'),
        DeclareLaunchArgument('odom_tf_angular_variance',   default_value='0.01',    description='If TF is used to get odometry, this is the default angular variance'),
        DeclareLaunchArgument('odom_tf_linear_variance',    default_value='0.001',   description='If TF is used to get odometry, this is the default linear variance'),
        DeclareLaunchArgument('odom_args',                  default_value='',      description='More arguments for odometry.'),
        DeclareLaunchArgument('odom_min_depth',             default_value='0.2',   description='Minimum depth in meters to use (0=disabled). Filters out closer objects.'),
        DeclareLaunchArgument('odom_sensor_sync',           default_value='false', description='Sensor sync for odometry.'),
        DeclareLaunchArgument('odom_guess_frame_id',        default_value='',      description='Frame id for odometry guess.'),
        DeclareLaunchArgument('odom_guess_min_translation', default_value='0.0',   description='Minimum translation for odometry guess.'),
        DeclareLaunchArgument('odom_guess_min_rotation',    default_value='0.0',   description='Minimum rotation for odometry guess.'),
        DeclareLaunchArgument('odom_always_process_most_recent_frame', default_value='true', description='Odometry: always process latest frame to reduce delay.'),
        DeclareLaunchArgument('odom_log_level',  default_value=LaunchConfiguration('log_level'), description='Specific ROS logger level for odometry node.'),

        # imu
        DeclareLaunchArgument('imu_topic',        default_value='/imu/data', description='IMU topic for odometry.'),
        DeclareLaunchArgument('wait_imu_to_init', default_value='false',     description='Wait for IMU to initialize.'),
        DeclareLaunchArgument('always_check_imu_tf', default_value='true',     description='Always check if TF between IMU frame and base frame has changed.'),
        OpaqueFunction(function=launch_setup)
    ])

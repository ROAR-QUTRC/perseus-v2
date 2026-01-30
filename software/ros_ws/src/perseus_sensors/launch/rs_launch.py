"""
RealSense Camera Launch File (+ Image Compression)

This launch file starts the RealSense2 camera driver (by including realsense2_camera/rs_launch.py)
and also launches image_transport republishers to generate compressed image topics.

Usage:
    ros2 launch perseus_sensors rs_launch.py

    Override topics if your RealSense topic names differ:
    ros2 launch perseus_sensors rs_launch.py \
        rgb_in:=/camera/camera/color/image_raw \
        depth_in:=/camera/camera/depth/image_rect_raw
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for RealSense2 camera with all parameters + compression republishers."""

    # Device selection parameters
    declare_args = [
        DeclareLaunchArgument('camera_name', default_value='camera', description='camera unique name'),
        DeclareLaunchArgument('camera_namespace', default_value='camera', description='namespace for camera'),
        DeclareLaunchArgument('serial_no', default_value="''", description='choose device by serial number'),
        DeclareLaunchArgument('usb_port_id', default_value="''", description='choose device by usb port id'),
        DeclareLaunchArgument('device_type', default_value="''", description='choose device by type'),
        DeclareLaunchArgument('config_file', default_value="''", description='yaml config file'),
        DeclareLaunchArgument('json_file_path', default_value="''", description='allows advanced configuration'),
        DeclareLaunchArgument('initial_reset', default_value='false', description='initial reset'),
        DeclareLaunchArgument('accelerate_gpu_with_glsl', default_value='false', description='enable GPU acceleration with GLSL'),
        DeclareLaunchArgument('rosbag_filename', default_value="''", description='A realsense bagfile to run from as a device'),
        DeclareLaunchArgument('rosbag_loop', default_value='false', description='Enable loop playback when playing a bagfile'),
        DeclareLaunchArgument('log_level', default_value='info', description='debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'),
        DeclareLaunchArgument('output', default_value='screen', description='pipe node output [screen|log]'),

        # Color stream parameters
        DeclareLaunchArgument('enable_color', default_value='true', description='enable color stream'),
        DeclareLaunchArgument('rgb_camera.color_profile', default_value='0,0,0', description='color stream profile'),
        DeclareLaunchArgument('rgb_camera.color_format', default_value='RGB8', description='color stream format'),
        DeclareLaunchArgument('rgb_camera.enable_auto_exposure', default_value='true', description='enable/disable auto exposure for color image'),

        # Depth stream parameters
        DeclareLaunchArgument('enable_depth', default_value='true', description='enable depth stream'),
        DeclareLaunchArgument('enable_infra', default_value='false', description='enable infra0 stream'),
        DeclareLaunchArgument('enable_infra1', default_value='false', description='enable infra1 stream'),
        DeclareLaunchArgument('enable_infra2', default_value='false', description='enable infra2 stream'),
        DeclareLaunchArgument('depth_module.depth_profile', default_value='0,0,0', description='depth stream profile'),
        DeclareLaunchArgument('depth_module.depth_format', default_value='Z16', description='depth stream format'),
        DeclareLaunchArgument('depth_module.infra_profile', default_value='0,0,0', description='infra streams (0/1/2) profile'),
        DeclareLaunchArgument('depth_module.infra_format', default_value='RGB8', description='infra0 stream format'),
        DeclareLaunchArgument('depth_module.infra1_format', default_value='Y8', description='infra1 stream format'),
        DeclareLaunchArgument('depth_module.infra2_format', default_value='Y8', description='infra2 stream format'),
        DeclareLaunchArgument('depth_module.color_profile', default_value='0,0,0', description='Depth module color stream profile for d405'),
        DeclareLaunchArgument('depth_module.color_format', default_value='RGB8', description='color stream format for d405'),

        # Depth module exposure and gain
        DeclareLaunchArgument('depth_module.exposure', default_value='8500', description='Depth module manual exposure value'),
        DeclareLaunchArgument('depth_module.gain', default_value='16', description='Depth module manual gain value'),
        DeclareLaunchArgument('depth_module.hdr_enabled', default_value='false', description='Depth module hdr enablement flag'),
        DeclareLaunchArgument('depth_module.enable_auto_exposure', default_value='true', description='enable/disable auto exposure for depth image'),
        DeclareLaunchArgument('depth_module.exposure.1', default_value='7500', description='Depth module first exposure value for hdr_merge'),
        DeclareLaunchArgument('depth_module.gain.1', default_value='16', description='Depth module first gain value for hdr_merge'),
        DeclareLaunchArgument('depth_module.exposure.2', default_value='1', description='Depth module second exposure value for hdr_merge'),
        DeclareLaunchArgument('depth_module.gain.2', default_value='16', description='Depth module second gain value for hdr_merge'),

        # Sync and IMU parameters
        DeclareLaunchArgument('enable_sync', default_value='false', description='enable sync mode'),
        DeclareLaunchArgument('depth_module.inter_cam_sync_mode', default_value='0', description='[0-Default, 1-Master, 2-Slave]'),
        DeclareLaunchArgument('enable_rgbd', default_value='false', description='enable rgbd topic'),
        DeclareLaunchArgument('enable_gyro', default_value='false', description='enable gyro stream'),
        DeclareLaunchArgument('enable_accel', default_value='false', description='enable accel stream'),
        DeclareLaunchArgument('gyro_fps', default_value='0', description='gyro fps'),
        DeclareLaunchArgument('enable_motion', default_value='false', description='enable motion stream (IMU) for DDS devices'),
        DeclareLaunchArgument('accel_fps', default_value='0', description='accel fps'),
        DeclareLaunchArgument('unite_imu_method', default_value='0', description='[0-None, 1-copy, 2-linear_interpolation]'),

        # Distance and covariance parameters
        DeclareLaunchArgument('clip_distance', default_value='-2.', description='clip distance'),
        DeclareLaunchArgument('angular_velocity_cov', default_value='0.01', description='angular velocity covariance'),
        DeclareLaunchArgument('linear_accel_cov', default_value='0.01', description='linear acceleration covariance'),

        # Publishing parameters
        DeclareLaunchArgument('diagnostics_period', default_value='0.0', description='Rate of publishing diagnostics. 0=Disabled'),
        DeclareLaunchArgument('publish_tf', default_value='true', description='enable/disable publishing static & dynamic TF'),
        DeclareLaunchArgument('tf_publish_rate', default_value='0.0', description='rate in Hz for publishing dynamic TF'),

        # Pointcloud parameters
        DeclareLaunchArgument('pointcloud.enable', default_value='false', description='enable pointcloud'),
        DeclareLaunchArgument('pointcloud.stream_filter', default_value='2', description='texture stream for pointcloud'),
        DeclareLaunchArgument('pointcloud.stream_index_filter', default_value='0', description='texture stream index for pointcloud'),
        DeclareLaunchArgument('pointcloud.ordered_pc', default_value='false', description='ordered pointcloud'),
        DeclareLaunchArgument('pointcloud.allow_no_texture_points', default_value='false', description='allow no texture points'),

        # Filter parameters
        DeclareLaunchArgument('align_depth.enable', default_value='false', description='enable align depth filter'),
        DeclareLaunchArgument('colorizer.enable', default_value='false', description='enable colorizer filter'),
        DeclareLaunchArgument('decimation_filter.enable', default_value='false', description='enable decimation filter'),
        DeclareLaunchArgument('decimation_filter.filter_magnitude', default_value='2', description='decimation filter magnitude'),
        DeclareLaunchArgument('rotation_filter.enable', default_value='false', description='enable rotation filter'),
        DeclareLaunchArgument('rotation_filter.rotation', default_value='0.0', description='rotation value: 0.0, 90.0, -90.0, 180.0'),
        DeclareLaunchArgument('spatial_filter.enable', default_value='false', description='enable spatial filter'),
        DeclareLaunchArgument('temporal_filter.enable', default_value='false', description='enable temporal filter'),
        DeclareLaunchArgument('disparity_filter.enable', default_value='false', description='enable disparity filter'),
        DeclareLaunchArgument('hole_filling_filter.enable', default_value='false', description='enable hole filling filter'),
        DeclareLaunchArgument('hdr_merge.enable', default_value='false', description='hdr_merge filter enablement flag'),

        # Connection parameters
        DeclareLaunchArgument('wait_for_device_timeout', default_value='-1.', description='Timeout for waiting for device to connect'),
        DeclareLaunchArgument('reconnect_timeout', default_value='6.', description='Timeout between consecutive reconnection attempts'),

        # Frame parameters
        DeclareLaunchArgument('base_frame_id', default_value='link', description='Root frame of the sensors transform tree'),
        DeclareLaunchArgument('tf_prefix', default_value='', description='prefix to be prepended to all frame IDs'),

        # Safety stream parameters
        DeclareLaunchArgument('enable_safety', default_value='false', description='enable safety stream'),
        DeclareLaunchArgument('safety_camera.safety_mode', default_value='0', description='[int] 0-Run, 1-Standby, 2-Service'),

        # Labeled pointcloud parameters
        DeclareLaunchArgument('enable_labeled_point_cloud', default_value='false', description='enable labeled point cloud stream'),
        DeclareLaunchArgument('depth_mapping_camera.labeled_point_cloud_profile', default_value='0,0,0', description='Label PointCloud stream profile'),

        # Occupancy parameters
        DeclareLaunchArgument('enable_occupancy', default_value='false', description='enable occupancy stream'),
        DeclareLaunchArgument('depth_mapping_camera.occupancy_profile', default_value='0,0,0', description='Occupancy stream profile'),

        # ---- Added: compression controls ----
        DeclareLaunchArgument(
            'enable_image_compression',
            default_value='true',
            description='If true, start image_transport republishers to create /compressed and /compressedDepth topics',
        ),
        DeclareLaunchArgument(
            'rgb_in',
            default_value='/camera/color/image_raw',
            description='Input RGB image topic to compress (raw)',
        ),
        DeclareLaunchArgument(
            'depth_in',
            default_value='/camera/depth/image_rect_raw',
            description='Input depth image topic to compress (raw)',
        ),
    ]

    # Include the realsense2_camera rs_launch.py with all parameters
    launch_arguments = {arg.name: LaunchConfiguration(arg.name) for arg in declare_args}

    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])
        ),
        launch_arguments=launch_arguments.items(),
    )

    # ---- Added: image_transport republish nodes ----
    # RGB: raw -> compressed (creates <rgb_in>/compressed)
    rgb_compress = Node(
        package='image_transport',
        executable='republish',
        name='rgb_compress_republish',
        output='screen',
        arguments=[
            'raw', 'compressed',
        ],
        remappings=[
            ('in', LaunchConfiguration('rgb_in')),
            ('out', LaunchConfiguration('rgb_in')),
        ],
        condition=None,  # keep simple; see note below
    )

    # Depth: raw -> compressedDepth (creates <depth_in>/compressedDepth)
    depth_compress = Node(
        package='image_transport',
        executable='republish',
        name='depth_compress_republish',
        output='screen',
        arguments=[
            'raw', 'compressedDepth',
        ],
        remappings=[
            ('in', LaunchConfiguration('depth_in')),
            ('out', LaunchConfiguration('depth_in')),
        ],
        condition=None,  # keep simple; see note below
    )

    ld = LaunchDescription()

    for arg in declare_args:
        ld.add_action(arg)

    ld.add_action(rs_launch)


    # we can add an IfCondition here. Keeping it always-on is simplest and works out of the box.
    ld.add_action(rgb_compress)
    ld.add_action(depth_compress)

    return ld
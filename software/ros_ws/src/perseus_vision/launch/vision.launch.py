from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time if true"
    )
    
    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # Cube detector node
    cube_detector_node = Node(
        package="perseus_vision",
        executable="cube_detector",
        name="cube_detector",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            # Add remappings if needed, e.g.:
            # ("/camera/image_raw", "/your_camera_topic"),
            # ("/detected_cubes", "/perception/cubes")
        ]
    )
    
    # ArUco detector node  
    aruco_detector_node = Node(
        package="perseus_vision",
        executable="aruco_detector", 
        name="aruco_detector",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            # Add remappings if needed, e.g.:
            # ("/camera/image_raw", "/your_camera_topic"),
            # ("/aruco_poses", "/perception/aruco_markers")
        ]
    )
    
    # Event handler to start cube detector only after aruco detector starts
    cube_detector_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=aruco_detector_node,
            on_start=[cube_detector_node]
        )
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        aruco_detector_node,
        cube_detector_event_handler,  # Launch cube detector when aruco detector starts
    ])
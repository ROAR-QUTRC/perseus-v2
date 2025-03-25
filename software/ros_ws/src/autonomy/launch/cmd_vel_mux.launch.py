from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Define launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Multiplexer configuration
    mux_node = Node(
        package='topic_tools',
        executable='mux',
        name='cmd_vel_mux',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            'cmd_vel',                                          # Output topic
            'cmd_vel_nav',                                      # Input from Nav2
            'diff_base_controller/cmd_vel',                     # Input from your controller
            '--timeout', '5.0',                                 # Timeout for switching
            '--default', 'diff_base_controller/cmd_vel'         # Default input topic
        ],
        remappings=[
            ('cmd_vel', '/cmd_vel'),                                 # Remap to robot's cmd_vel
            ('cmd_vel_nav', '/nav2/cmd_vel'),                        # Nav2 output
            ('diff_base_controller/cmd_vel', '/controller/cmd_vel')  # Your controller
        ]
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        
        mux_node
    ])

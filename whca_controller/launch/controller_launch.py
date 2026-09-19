from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Define the launch configurations (to capture terminal inputs)
    safeguards = LaunchConfiguration('safeguards')
    k_robust = LaunchConfiguration('k_robust')
    debug = LaunchConfiguration('debug')

    # 2. Declare the launch arguments with default values and descriptions
    declare_safeguards = DeclareLaunchArgument(
        'safeguards',
        default_value='false',
        description='Toggle for safety override systems (Boolean: true/false)'
    )

    declare_k_robust = DeclareLaunchArgument(
        'k_robust',
        default_value='0',
        description='Robustness factor parameter for the controller (Integer)'
    )
    
    declare_debug = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable verbose debug output'
    )

    # 3. Define the node that will consume these parameters
    robot_controller_node = Node(
        package='whca_controller',      # Replace with your package name
        executable='whca_controller',   # Replace with your node executable name
        name='whca_controller_node',
        output='screen',
        parameters=[{
            'safeguards': safeguards,
            'k_robust': k_robust,
            'debug': debug,
            'use_sim_time': True
        }]
    )

    # 4. Return the LaunchDescription containing all arguments and nodes
    return LaunchDescription([
        declare_safeguards,
        declare_k_robust,
        declare_debug,
        robot_controller_node
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Define the launch configurations (to capture terminal inputs)
    num_robots = LaunchConfiguration('num_robots')
    safeguards_enabled = LaunchConfiguration('safeguards')
    k_robust = LaunchConfiguration('k_robust')

    # 2. Declare the launch arguments with default values and descriptions
    declare_num_robots = DeclareLaunchArgument(
        'num_robots',
        default_value='20',
        description='Number of active robots in the swarm (Integer)'
    )

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

    # 3. Define the node that will consume these parameters
    robot_controller_node = Node(
        package='whca_controller',      # Replace with your package name
        executable='whca_controller',   # Replace with your node executable name
        name='whca_controller_node',
        output='screen',
        parameters=[{
            'num_robots': num_robots,
            'safeguards': safeguards_enabled,
            'k_robust': k_robust
        }]
    )

    # 4. Return the LaunchDescription containing all arguments and nodes
    return LaunchDescription([
        declare_num_robots,
        declare_safeguards,
        declare_k_robust,
        robot_controller_node
    ])

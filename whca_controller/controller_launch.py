from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. Define the launch configurations (to capture terminal inputs)
    safeguards = LaunchConfiguration('safeguards')
    k_robust = LaunchConfiguration('k_robust')
    debug = LaunchConfiguration('debug')
    sync_mode = LaunchConfiguration('sync_mode')
    step_seconds = LaunchConfiguration('step_seconds')
    barrier_timeout = LaunchConfiguration('barrier_timeout')
    num_robots = LaunchConfiguration('num_robots')

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

    declare_sync_mode = DeclareLaunchArgument(
        'sync_mode', default_value='barrier', choices=['barrier', 'clock'])
    declare_step_seconds = DeclareLaunchArgument('step_seconds', default_value='3.5')
    declare_barrier_timeout = DeclareLaunchArgument(
        'barrier_timeout', default_value='0.0',
        description='0 disables timeout; positive simulation seconds stop the run on expiry')
    declare_num_robots = DeclareLaunchArgument(
        'num_robots', default_value='20',
        description='Must match launch_isaac.py --num_robots; waits for this complete fleet')

    # 3. Define the node that will consume these parameters
    robot_controller_node = Node(
        package='whca_controller',      # Replace with your package name
        executable='whca_controller',   # Replace with your node executable name
        name='whca_controller_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'safeguards': safeguards,
            'k_robust': k_robust,
            'debug': debug,
            'sync_mode': sync_mode,
            'step_seconds': ParameterValue(step_seconds, value_type=float),
            'barrier_timeout': ParameterValue(barrier_timeout, value_type=float),
            'num_robots': ParameterValue(num_robots, value_type=int),
            'use_sim_time': True
        }]
    )

    # 4. Return the LaunchDescription containing all arguments and nodes
    return LaunchDescription([
        declare_safeguards,
        declare_k_robust,
        declare_debug,
        declare_sync_mode,
        declare_step_seconds,
        declare_barrier_timeout,
        declare_num_robots,
        robot_controller_node
    ])

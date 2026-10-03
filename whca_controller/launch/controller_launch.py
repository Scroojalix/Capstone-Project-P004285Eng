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
    density_scaling = LaunchConfiguration('density_scaling')
    sync_mode = LaunchConfiguration('sync_mode')
    step_seconds = LaunchConfiguration('step_seconds')
    barrier_timeout = LaunchConfiguration('barrier_timeout')
    window_size = LaunchConfiguration('window_size')
    seed = LaunchConfiguration('seed')
    starvation_priority = LaunchConfiguration('starvation_priority')
    results_file = LaunchConfiguration('results_file')

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
    
    declare_density_scaling = DeclareLaunchArgument(
        'density_scaling', default_value='true',
        description='Enable density-based speed scaling'
    )
    
    declare_step_seconds = DeclareLaunchArgument('step_seconds', default_value='3.5')
    declare_barrier_timeout = DeclareLaunchArgument(
        'barrier_timeout', default_value='0.0',
        description='0 disables timeout; positive simulation seconds stop the run on expiry')

    declare_window_size = DeclareLaunchArgument(
        'window_size', default_value='32',
        description='WHCA* window W; the first W//2 steps are committed each window')
    declare_seed = DeclareLaunchArgument(
        'seed', default_value='1',
        description='Fixes goal assignment and per-window priority. MUST equal '
                    'launch_isaac.py --seed so spawns match.')
    declare_starvation_priority = DeclareLaunchArgument(
        'starvation_priority', default_value='true',
        description='Plan robots that made no progress last window first')
    declare_results_file = DeclareLaunchArgument(
        'results_file', default_value='whca_results.csv',
        description='CSV file each run appends one row of metrics to')

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
            'density_scaling': density_scaling,
            'sync_mode': sync_mode,
            'step_seconds': ParameterValue(step_seconds, value_type=float),
            'barrier_timeout': ParameterValue(barrier_timeout, value_type=float),
            'window_size': ParameterValue(window_size, value_type=int),
            'seed': ParameterValue(seed, value_type=int),
            'starvation_priority': ParameterValue(starvation_priority, value_type=bool),
            'results_file': results_file,
            'use_sim_time': True
        }]
    )

    # 4. Return the LaunchDescription containing all arguments and nodes
    return LaunchDescription([
        declare_safeguards,
        declare_k_robust,
        declare_debug,
        declare_sync_mode,
        declare_density_scaling,
        declare_step_seconds,
        declare_barrier_timeout,
        declare_window_size,
        declare_seed,
        declare_starvation_priority,
        declare_results_file,
        robot_controller_node
    ])

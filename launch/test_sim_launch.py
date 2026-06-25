import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    package_path = FindPackageShare('gazebo_test')
    
    # World file location
    world_file = PathJoinSubstitution([
        package_path,
        'worlds/test_sim.sdf'
    ])
    
    # Find gz_sim launch file
    gz_launch_path = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    ])
    
    # Gazebo simulation launch with world file argument
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': world_file,
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Locate bridge yaml file
    bridge_yaml_file = os.path.join(
        get_package_share_directory('gazebo_test'),
        'config',
        'gz_bridge.yaml'
    )
    
    # Parameter bridge node
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_param_bridge',
        parameters=[{
            'config_file': bridge_yaml_file
        }],
        output='screen'
    )
    
    return LaunchDescription([
        gz_sim,
        bridge_node,
        Node(
            package='gazebo_test',
            executable='my_test_node',
            name='gazebo_test_node',
        )
    ])
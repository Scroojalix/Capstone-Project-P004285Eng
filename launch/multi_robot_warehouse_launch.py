import os
import tempfile
import yaml

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, LogInfo, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnShutdown

def gen_robot_list(num_robots):
    robots = []
    row_length = 20
    
    for i in range(num_robots):
        robot_name = f'robot{i}'    
        
        row = i // row_length
        col = i % row_length        
        
        x_pos = -row * 4.0
        y_pos = (row_length/2 - col) * 2.0
        
        robot = {
            'name': robot_name,
            'x': x_pos,
            'y': y_pos,
            'z': 0.0
        }
        robots.append(robot)
        
    return robots


def generate_launch_description():
    
    pkg_share = get_package_share_directory('gazebo_test')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    package_path = FindPackageShare('gazebo_test')
    
    # World file location
    world_name = 'warehouse'
    world_file = PathJoinSubstitution([package_path, f'worlds/{world_name}.sdf'])
    
    # Find gz_sim launch file
    gz_launch_path = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    
    # Gazebo simulation launch with world file argument
    gz_sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': world_file,
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Create the launch description
    ld = LaunchDescription([gz_sim_node])
    
    # Get the path to the robot model SDF file
    sdf_path = os.path.join(pkg_share, 'models', 'robot.sdf')
    assert os.path.exists(sdf_path), f"Model SDF file does not exist: {sdf_path}"
    
    # Generate robot spawn nodes
    robots = gen_robot_list(200)
    
    bridge_mappings = []
    
    last_robot_spawn_node = None
    for i, robot in enumerate(robots):
        
        robot_name = robot['name']
                
        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', robot_name,
                '-file', sdf_path,
                '-x', str(robot['x']),
                '-y', str(robot['y']),
                '-z', str(robot['z']),
            ],
            output='screen'
        )
        
        ld.add_action(TimerAction(period=i*0.2+5.0, actions=[spawn_node]))
        
        last_robot_spawn_node = spawn_node
        
        # Add bridge mappings for each robot
        bridge_mappings.extend([{
            'ros_topic_name': f'/{robot_name}/cmd_vel',
            'gz_topic_name': f'/model/{robot_name}/cmd_vel',
            'ros_type_name': 'geometry_msgs/msg/Twist',
            'gz_type_name': 'gz.msgs.Twist',
            'direction': 'ROS_TO_GZ'
        },
        {
            'ros_topic_name': f'/{robot_name}/imu',
            'gz_topic_name': f'/world/{world_name}/model/{robot_name}/link/chassis/sensor/imu_sensor/imu',
            'ros_type_name': 'sensor_msgs/msg/Imu',
            'gz_type_name': 'gz.msgs.IMU',
            'direction': 'GZ_TO_ROS'
        },
        {
            'ros_topic_name': f'/{robot_name}/lidar',
            'gz_topic_name': f'/world/{world_name}/model/{robot_name}/link/chassis/sensor/gpu_lidar/scan',
            'ros_type_name': 'sensor_msgs/msg/LaserScan',
            'gz_type_name': 'gz.msgs.LaserScan',
            'direction': 'GZ_TO_ROS'
        },
        {
            'ros_topic_name': f'/{robot_name}/odom',
            'gz_topic_name': f'/model/{robot_name}/odometry',
            'ros_type_name': 'nav_msgs/msg/Odometry',
            'gz_type_name': 'gz.msgs.Odometry',
            'direction': 'GZ_TO_ROS'
        },
        {
            'ros_topic_name': f'/{robot_name}/tf',
            'gz_topic_name': f'/model/{robot_name}/tf',
            'ros_type_name': 'geometry_msgs/msg/PoseArray',
            'gz_type_name': 'gz.msgs.Pose_V',
            'direction': 'GZ_TO_ROS'
        }
        ])
        
    # 4. Write the built python dictionary out to a temporary YAML file
    # Using delete=False guarantees the file stays active while the node reads it
    tmp_config = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(bridge_mappings, tmp_config, default_flow_style=False)
    tmp_config.close()
        
    # Locate bridge yaml file
    # bridge_yaml_file = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    
    # Parameter bridge node
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_param_bridge',
        parameters=[{
            'config_file': tmp_config.name
        }],
        output='screen'
    )
        
    ld.add_action(bridge_node)
    
    mapf_node = Node(
        package='gazebo_test',
        executable='mapf_node',
        name='my_mapf_node',
    )
    
    wait_for_robots_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=last_robot_spawn_node,
            on_exit=[
                LogInfo(msg="All robots have been spawned. Starting MAPF node..."),
                mapf_node
            ]
        )
    )
    
    ld.add_action(wait_for_robots_node)
    
    # Clean up the temporary file automatically when the launch ends
    ld.add_action(RegisterEventHandler(
        OnShutdown(on_shutdown=[lambda: os.unlink(tmp_config.name)])
    ))
    
    return ld
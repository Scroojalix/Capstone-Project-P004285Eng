"""Multi-robot warehouse launch for the WHCA Gazebo demo.

Changes:
  * Robot count is now a CLI argument (default 30, matching the proposal's
    fleet size) instead of a hard-coded 200.
  * Only the bridges actually used downstream are created: cmd_vel (ROS->GZ)
    and odom (GZ->ROS). The old imu/lidar/tf bridges either pointed at topics
    the current robot.sdf never publishes (lidar, tf) or weren't consumed,
    so they were pure overhead.
  * `headless` and `run` flags are threaded into gz_args so you can run the
    server with no GUI (big speed win for batch runs) and auto-start the sim
    without pressing play.

Usage:
    ros2 launch gazebo_test multi_robot_warehouse_launch.py
    ros2 launch gazebo_test multi_robot_warehouse_launch.py num_robots:=10
    ros2 launch gazebo_test multi_robot_warehouse_launch.py headless:=true num_robots:=30
"""

import os
import tempfile
import yaml

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, RegisterEventHandler, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnShutdown


# --- Grid layout. MUST match the MAPF node's grid params (mapf.py) ---------
# Robots spawn at cell centres of a 3 m grid so each gets a unique start cell

GRID_RES = 3.0
ORIGIN_X = -30.0
ORIGIN_Y = -30.0
START_CX = 2        # left start block, column 0
TOP_CY = 17         # top row of the start block
ROWS_PER_COL = 15   # robots stacked per column before wrapping to next column


def _cell_center(cx, cy):
    return (ORIGIN_X + (cx + 0.5) * GRID_RES,
            ORIGIN_Y + (cy + 0.5) * GRID_RES)


def gen_robot_list(num_robots):
    """Grid-aligned spawn block: one robot per cell, left side of the map."""
    robots = []
    for i in range(num_robots):
        col = i // ROWS_PER_COL
        row = i % ROWS_PER_COL
        cx = START_CX + col
        cy = TOP_CY - row
        x, y = _cell_center(cx, cy)
        robots.append({'name': f'robot{i}', 'x': x, 'y': y, 'z': 0.0})
    return robots


def launch_setup(context, *args, **kwargs):
    """Resolved at launch time so num_robots/headless/run become real values."""
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    run = LaunchConfiguration('run').perform(context).lower() == 'true'
    world_name = LaunchConfiguration('world').perform(context)

    pkg_share = get_package_share_directory('gazebo_test')
    world_file = os.path.join(pkg_share, 'worlds', f'{world_name}.sdf')
    assert os.path.exists(world_file), f"World file does not exist: {world_file}"

    sdf_path = os.path.join(pkg_share, 'models', 'robot.sdf')
    assert os.path.exists(sdf_path), f"Model SDF file does not exist: {sdf_path}"

    # Assemble gz_args: flags first, then the world file.
    #   -s : server only (headless, no GUI render load)
    #   -r : run immediately (no need to press play)
    gz_flags = []
    if headless:
        gz_flags.append('-s')
    if run:
        gz_flags.append('-r')
    gz_args = ' '.join(gz_flags + [world_file])

    gz_launch_path = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])

    gz_sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': gz_args,
            'on_exit_shutdown': 'true',
        }.items()
    )

    actions = [gz_sim_node]

    robots = gen_robot_list(num_robots)
    bridge_mappings = []
    last_robot_spawn_node = None

    for i, robot in enumerate(robots):
        name = robot['name']

        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', name,
                '-file', sdf_path,
                '-x', str(robot['x']),
                '-y', str(robot['y']),
                '-z', str(robot['z']),
            ],
            output='screen',
        )
        # Stagger spawns so gz isn't asked to insert 30 models in one tick.
        actions.append(TimerAction(period=i * 0.2 + 5.0, actions=[spawn_node]))
        last_robot_spawn_node = spawn_node

        # Only the two bridges we actually use.
        bridge_mappings.extend([
            {
                'ros_topic_name': f'/{name}/cmd_vel',
                'gz_topic_name': f'/model/{name}/cmd_vel',
                'ros_type_name': 'geometry_msgs/msg/Twist',
                'gz_type_name': 'gz.msgs.Twist',
                'direction': 'ROS_TO_GZ',
            },
            {
                'ros_topic_name': f'/{name}/odom',
                'gz_topic_name': f'/model/{name}/odometry',
                'ros_type_name': 'nav_msgs/msg/Odometry',
                'gz_type_name': 'gz.msgs.Odometry',
                'direction': 'GZ_TO_ROS',
            },
            {
                # Ground-truth world pose from the PosePublisher plugin in
                # robot.sdf. Pose_V -> PoseArray; the controller reads poses[0]
                # as the model's world pose. This is what WHCA uses for the
                # global grid (odom alone is only relative to spawn).
                'ros_topic_name': f'/{name}/pose',
                'gz_topic_name': f'/model/{name}/pose',
                'ros_type_name': 'geometry_msgs/msg/PoseArray',
                'gz_type_name': 'gz.msgs.Pose_V',
                'direction': 'GZ_TO_ROS',
            },
        ])

    tmp_config = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False)
    yaml.dump(bridge_mappings, tmp_config, default_flow_style=False)
    tmp_config.close()

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_param_bridge',
        parameters=[{'config_file': tmp_config.name}],
        output='screen',
    )
    actions.append(bridge_node)

    # Start the MAPF controller only once the final robot has spawned.
    mapf_node = Node(
        package='gazebo_test',
        executable='mapf_node',
        name='my_mapf_node',
        output='screen',
        parameters=[{'num_robots': num_robots}],
    )
    actions.append(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=last_robot_spawn_node,
            on_exit=[
                LogInfo(msg="All robots spawned. Starting MAPF node..."),
                mapf_node,
            ],
        )
    ))

    actions.append(RegisterEventHandler(
        OnShutdown(on_shutdown=[lambda *a, **k: os.unlink(tmp_config.name)])
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='30',
                              description='Number of robots to spawn.'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run gz server with no GUI (-s).'),
        DeclareLaunchArgument('run', default_value='true',
                              description='Auto-start the sim (-r), no play press.'),
        DeclareLaunchArgument('world', default_value='warehouse',
                              description='World file stem under worlds/.'),
        OpaqueFunction(function=launch_setup),
    ])

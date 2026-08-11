import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('whca_rviz_baseline')
    rviz_config_file = os.path.join(pkg_share, 'config', 'whca_baseline_rviz_config.rviz')
    
    # RViz visualization node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    
    # WHCA experiment node
    whca_node = Node(
        package='whca_rviz_baseline',
        executable='whca_rviz_baseline_node',
        name='whca_rviz_baseline_node',
        output='screen'
    )

    return LaunchDescription([
        rviz_node,
        whca_node
    ])
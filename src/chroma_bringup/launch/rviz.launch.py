"""
rviz.launch.py
Starts RViz with CHROMA dashboard panels and config.

Author: H.A. Sharif
Year: 2026
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    chroma_sim_dir = get_package_share_directory('chroma_simulation')
    chroma_bringup_dir = get_package_share_directory('chroma_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config_file = os.path.join(chroma_sim_dir, 'rviz', 'chroma_config.rviz')
    tf_aggregator_file = os.path.join(chroma_bringup_dir, 'launch', 'tf_aggregator.launch.py')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    tf_aggregator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tf_aggregator_file)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}] 
    )

    return LaunchDescription([
        use_sim_time_arg,
        tf_aggregator_launch,
        rviz_node
    ])
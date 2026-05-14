import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    cares_sim_dir = get_package_share_directory('cares_simulation')
    cares_bringup_dir = get_package_share_directory('cares_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config_file = os.path.join(cares_sim_dir, 'rviz', 'cares_config.rviz')
    tf_aggregator_file = os.path.join(cares_bringup_dir, 'launch', 'tf_aggregator.launch.py')

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
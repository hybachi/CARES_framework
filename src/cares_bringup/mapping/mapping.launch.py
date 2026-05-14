import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('cares_bringup')
    robot_id = LaunchConfiguration('robot_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    mapping_params = os.path.join(bringup_dir, 'mapping', 'mapping_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('robot_id',     default_value='tb3_0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='cares_bringup',
            executable='scan_resampler',
            name='scan_resampler',
            namespace=robot_id,
            parameters=[{
                'num_ranges': 360,
                'angle_min':  0.0,
                'angle_max':  6.28318,
            }],
            output='screen'
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                mapping_params,
                {'use_sim_time': use_sim_time}
            ],
                remappings=[
                ('/scan', '/tb3_0/scan_fixed')
            ]
        ),
    ])
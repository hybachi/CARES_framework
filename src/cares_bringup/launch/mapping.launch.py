import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('cares_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')

    mapping_params = os.path.join(bringup_dir, 'config', 'slam', 'mapping_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            mapping_params,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/tb3_0/scan')
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_toolbox_node
    ])
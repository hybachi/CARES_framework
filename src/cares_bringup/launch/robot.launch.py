import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    robot_id = LaunchConfiguration('robot_id')
    
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(get_package_share_directory('cares_bringup'), 'config', 'tb3_profile.yaml'),
        description='Path to the robot profile YAML file'
    )

    robot_id_arg = DeclareLaunchArgument(
        'robot_id', 
        description='Unique ID and namespace for the robot'
    )

    capability_manager = Node(
        package='cares_core',
        executable='capability_manager',
        name='capability_manager',
        namespace=LaunchConfiguration('robot_id', default='tb3_0'),
        parameters=[
            config_file, 
            {'robot_id': robot_id}
        ],
        output='screen'
    )

    task_allocator = Node(
        package='cares_core',
        executable='task_allocator',
        name='task_allocator',
        namespace=LaunchConfiguration('robot_id', default='tb3_0'),
        parameters=[
            config_file, 
            {'robot_id': robot_id}
        ],
        output='screen'
    )

    return LaunchDescription([
        config_arg,
        robot_id_arg,
        capability_manager,
        task_allocator
    ])
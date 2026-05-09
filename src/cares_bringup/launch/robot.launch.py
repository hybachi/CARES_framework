from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node

def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    robot_id = LaunchConfiguration('robot_id')

    args = [
        DeclareLaunchArgument('config_file', description='Path to profile YAML'),
        DeclareLaunchArgument('robot_id', description='Robot ID / Namespace'),
        DeclareLaunchArgument('execution_interface', default_value='cmd_vel', description='cmd_vel, nav2, none'),
        DeclareLaunchArgument('telemetry_interface', default_value='standard', description='standard, none')
    ]

    core_nodes = [
        Node(package='cares_core', executable='capability_manager', namespace=robot_id, parameters=[config_file, {'robot_id': robot_id}], output='screen'),
        Node(package='cares_core', executable='task_allocator', namespace=robot_id, parameters=[config_file, {'robot_id': robot_id}], output='screen')
    ]

    bridge_nodes = [
        Node(package='cares_bridges', executable='cmd_vel_bridge', namespace=robot_id, parameters=[{'robot_id': robot_id}],
             condition=LaunchConfigurationEquals('execution_interface', 'cmd_vel')),
        Node(package='cares_bridges', executable='nav2_bridge', namespace=robot_id, parameters=[{'robot_id': robot_id}],
             condition=LaunchConfigurationEquals('execution_interface', 'nav2')),
        Node(package='cares_bridges', executable='telemetry_bridge', namespace=robot_id, parameters=[config_file],
             condition=LaunchConfigurationEquals('telemetry_interface', 'standard'))
    ]

    return LaunchDescription(args + core_nodes + bridge_nodes)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    config_name = LaunchConfiguration('config_name')
    
    config_path = PathJoinSubstitution([
        FindPackageShare('cares_bringup'),
        'config',
        config_name
    ])
    
    args = [
        DeclareLaunchArgument(
            'config_name', 
            default_value='tb3_profile.yaml', 
            description='Name of the YAML file in cares_bringup/config/'
        ),
        DeclareLaunchArgument('robot_id', default_value='tb3_0', description='Robot ID / Namespace'),
        DeclareLaunchArgument('execution_interface', default_value='cmd_vel', description='cmd_vel, nav2, none'),
        DeclareLaunchArgument('telemetry_interface', default_value='standard', description='standard, none')
    ]

    core_nodes = [
        Node(
            package='cares_core', executable='capability_manager', namespace=robot_id, 
            parameters=[config_path, {'robot_id': robot_id}], output='screen'
        ),
        Node(
            package='cares_core', executable='task_allocator', namespace=robot_id, 
            parameters=[config_path, {'robot_id': robot_id}], output='screen'
        )
    ]

    bridge_nodes = [
        Node(
            package='cares_bridges', executable='cmd_vel_bridge', namespace=robot_id, 
            parameters=[{'robot_id': robot_id}], condition=LaunchConfigurationEquals('execution_interface', 'cmd_vel')
        ),
        Node(
            package='cares_bridges', executable='nav2_bridge', namespace=robot_id, 
            parameters=[{'robot_id': robot_id}], condition=LaunchConfigurationEquals('execution_interface', 'nav2')
        ),
        Node(
            package='cares_bridges', executable='telemetry_bridge', namespace=robot_id, 
            parameters=[config_path, {'robot_id': robot_id}], condition=LaunchConfigurationEquals('telemetry_interface', 'standard')
        )
    ]

    return LaunchDescription(args + core_nodes + bridge_nodes)
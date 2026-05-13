import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    tb3_bringup = get_package_share_directory('turtlebot3_bringup')
    tb3_description = get_package_share_directory('turtlebot3_description')
    lidar_pkg = get_package_share_directory('ld08_driver')

    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    robot_id = LaunchConfiguration('robot_id')

    tb3_param  = os.path.join(tb3_bringup, 'param', 'humble', f'{TURTLEBOT3_MODEL}.yaml')
    urdf       = os.path.join(tb3_description, 'urdf', f'turtlebot3_{TURTLEBOT3_MODEL}.urdf')
    robot_desc = Command(['xacro ', urdf, ' namespace:=', robot_id, '/'])

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='tb3_0'),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_id,
            parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen'
        ),

        # LDS-02 LiDAR
        Node(
            package='ld08_driver',
            executable='ld08_driver',
            name='ld08_driver',
            namespace=robot_id,
            parameters=[{'frame_id': [robot_id, '/base_scan']}],
            output='screen'
        ),

        # TurtleBot3 Node
        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            namespace=robot_id,
            parameters=[
                tb3_param,
                {
                    'namespace': robot_id,  
                    'diff_drive_controller.odometry.frame_id':       [robot_id, '/odom'],
                    'diff_drive_controller.odometry.child_frame_id': [robot_id, '/base_footprint'],
                }
            ],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen'
        ),
    ])
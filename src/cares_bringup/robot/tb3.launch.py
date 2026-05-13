import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    tb3_bringup = get_package_share_directory('turtlebot3_bringup')
    robot_id = LaunchConfiguration('robot_id')

    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='tb3_0')

    hardware_group = GroupAction(
        actions=[
            PushRosNamespace(robot_id),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tb3_bringup, 'launch', 'robot.launch.py')
                ),
                launch_arguments={
                    'namespace': robot_id,
                }.items()
            ),
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    return LaunchDescription([
        declare_robot_id,
        hardware_group
    ])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    cares_bringup_dir = get_package_share_directory('cares_bringup')
    tb3_bringup_dir = get_package_share_directory('turtlebot3_bringup')

    robot_id = LaunchConfiguration('robot_id')
    config_name = LaunchConfiguration('config_name')

    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='tb3_0')
    declare_config_name = DeclareLaunchArgument('config_name', default_value='tb3_profile.yaml')

    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    tb3_base = GroupAction([
        PushRosNamespace(robot_id),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3_bringup_dir, 'launch', 'robot.launch.py'))
        )
    ])

    cares_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cares_bringup_dir, 'launch', 'robot.launch.py')),
        launch_arguments={
            'robot_id': robot_id,
            'config_name': config_name,
            'execution_interface': 'nav2',      
            'telemetry_interface': 'standard'  
        }.items()
    )

    return LaunchDescription([
        set_tb3_model,
        declare_robot_id,
        declare_config_name,
        tb3_base,
        cares_stack
    ])
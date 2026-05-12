import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetParameter, SetRemap

def generate_launch_description():
    tb3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    robot_id = LaunchConfiguration('robot_id')

    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='tb3_0')

    hardware_group = GroupAction([
        PushRosNamespace(robot_id),
        SetParameter(name='frame_prefix', value=[robot_id, '/']),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3_bringup_dir, 'launch', 'robot.launch.py'))
        )
    ])

    return LaunchDescription([
        declare_robot_id,
        hardware_group
    ])
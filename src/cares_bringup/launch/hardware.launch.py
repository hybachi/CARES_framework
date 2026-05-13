import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bringup_dir = get_package_share_directory('cares_bringup')

    args = [
        DeclareLaunchArgument('robot_id',    default_value='tb3_0'),
        DeclareLaunchArgument('config_name', default_value='tb3_profile.yaml'),
    ]

    def launch_hardware(context, *args, **kwargs):
        robot_id = context.launch_configurations['robot_id']
        config_name = context.launch_configurations['config_name']
        config_path = os.path.join(bringup_dir, 'config', config_name)

        with open(config_path, 'r') as f:
            profile = yaml.safe_load(f)
        params = profile.get('/**', {}).get('ros__parameters', {})
        hw = params.get('hardware', {})
        nav2 = params.get('nav2',     {})

        actions = []

        for driver in hw.get('drivers', []):
            pkg         = driver['package']
            launch_file = driver['launch']
            raw_args    = driver.get('args', {})
            resolved    = {
                k: v.replace('ROBOT_ID', robot_id)
                for k, v in raw_args.items()
            }

            try:
                pkg_dir = get_package_share_directory(pkg)
            except Exception:
                print(f"[hardware.launch] WARNING: package '{pkg}' not found, skipping driver")
                continue

            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_dir, 'launch', launch_file)
                    ),
                    launch_arguments=resolved.items()
                )
            )

        pose = nav2.get('initial_pose', {})
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'robot.launch.py')
                ),
                launch_arguments={
                    'config_name': config_name,
                    'robot_id': robot_id,
                    'execution_interface': params.get('execution_interface',  'nav2'),
                    'telemetry_interface': params.get('telemetry_interface',  'standard'),
                    'initial_x': str(pose.get('x', 0.0)),
                    'initial_y': str(pose.get('y', 0.0)),
                    'initial_yaw': str(pose.get('yaw_degrees', 0.0)),
                }.items()
            )
        )

        return actions

    return LaunchDescription(args + [OpaqueFunction(function=launch_hardware)])
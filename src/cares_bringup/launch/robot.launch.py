import os
import yaml
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_id    = LaunchConfiguration('robot_id')
    config_name = LaunchConfiguration('config_name')
    config_path = PathJoinSubstitution([FindPackageShare('cares_bringup'), 'config', config_name])

    args = [
        DeclareLaunchArgument('config_name', default_value='tb3_profile.yaml'),
        DeclareLaunchArgument('robot_id', default_value='tb3_0'),
        DeclareLaunchArgument('execution_interface', default_value='nav2', description='cmd_vel | nav2 | none'),
        DeclareLaunchArgument('telemetry_interface', default_value='standard', description='standard | none'),
        # Pose overrides
        DeclareLaunchArgument('initial_x', default_value=''),
        DeclareLaunchArgument('initial_y', default_value=''),
        DeclareLaunchArgument('initial_yaw', default_value=''),
    ]

    core_nodes = [
        Node(
            package='cares_core', executable='capability_manager',
            namespace=robot_id, name='capability_manager',
            parameters=[config_path, {'robot_id': robot_id}], output='screen'
        ),
        Node(
            package='cares_core', executable='task_allocator',
            namespace=robot_id, name='task_allocator',
            parameters=[config_path, {'robot_id': robot_id}], output='screen'
        ),
    ]

    bridge_nodes = [
        Node(
            package='cares_bridges', executable='cmd_vel_bridge',
            namespace=robot_id, parameters=[{'robot_id': robot_id}],
            condition=LaunchConfigurationEquals('execution_interface', 'cmd_vel')
        ),
        Node(
            package='cares_bridges', executable='nav2_bridge',
            namespace=robot_id, parameters=[{'robot_id': robot_id}],
            condition=LaunchConfigurationEquals('execution_interface', 'nav2')
        ),
        Node(
            package='cares_bridges', executable='telemetry_bridge',
            namespace=robot_id,
            parameters=[config_path, {'robot_id': robot_id}],
            condition=LaunchConfigurationEquals('telemetry_interface', 'standard')
        ),
    ]

    # OpaqueFunction reads the profile and conditionally adds:
    #   Hardware bringup (physical only)
    #   Nav2 stack (if nav2.enabled)
    def launch_optional_stacks(context, *args, **kwargs):
        bringup_dir = get_package_share_directory('cares_bringup')
        cfg_name = context.launch_configurations.get('config_name', 'tb3_profile.yaml')
        cfg_path = os.path.join(bringup_dir, 'config', cfg_name)
        robot_id_val = context.launch_configurations.get('robot_id', 'tb3_0')

        with open(cfg_path, 'r') as f:
            profile = yaml.safe_load(f)
        params = profile.get('/**', {}).get('ros__parameters', {})

        actions = []

        # Hardware bringup
        hw = params.get('hardware', {})
        hw_pkg = hw.get('bringup_package', '')
        hw_launch = hw.get('bringup_launch', '')

        if hw_pkg and hw_launch:
            try:
                hw_dir = get_package_share_directory(hw_pkg)
                actions.append(
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(hw_dir, 'launch', hw_launch)
                        )
                    )
                )
            except Exception as e:
                # Package not installed
                pass

        # Nav2 stack 
        nav2_cfg = params.get('nav2', {})
        if nav2_cfg.get('enabled', False):
            nav2_params_fn = nav2_cfg.get('params_file', 'nav2_params.yaml')
            nav2_params_fp = os.path.join(bringup_dir, 'config', 'nav2', nav2_params_fn)
            use_sim_time = str(nav2_cfg.get('use_sim_time', False)).lower()

            map_name  = nav2_cfg.get('map_yaml', 'empty_map.yaml') 
            maps_dir  = os.path.join(get_package_share_directory('cares_bringup'), 'config', 'maps')
            map_yaml  = os.path.join(maps_dir, map_name)

            with open(map_yaml, 'r') as f:
                map_data = yaml.safe_load(f)

            map_data['image'] = os.path.join(maps_dir, map_data['image'])  # make absolute

            tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
            yaml.dump(map_data, tmp)
            tmp.close()
            map_yaml = tmp.name

            pose_cfg = nav2_cfg.get('initial_pose', {})
            # CLI overrides win over profile defaults
            x = context.launch_configurations.get('initial_x') or str(pose_cfg.get('x', 0.0))
            y = context.launch_configurations.get('initial_y') or str(pose_cfg.get('y', 0.0))
            yaw = context.launch_configurations.get('initial_yaw') or str(pose_cfg.get('yaw_degrees', 0.0))
            cov = pose_cfg.get('covariance_level', 'approximate')

            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_dir, 'launch', 'nav2.launch.py')
                    ),
                    launch_arguments={
                        'robot_id': robot_id_val,
                        'map_yaml': map_yaml,
                        'nav2_params': nav2_params_fp,
                        'use_sim_time': use_sim_time,
                        'initial_x': x,
                        'initial_y': y,
                        'initial_yaw': yaw,
                        'covariance_level': cov,
                    }.items()
                )
            )

        return actions

    return LaunchDescription(
        args + core_nodes + bridge_nodes + [OpaqueFunction(function=launch_optional_stacks)]
    )
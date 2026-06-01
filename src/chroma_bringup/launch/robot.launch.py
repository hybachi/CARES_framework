"""
robot.launch.py
Spawns the CHROMA core nodes and hardware bridges.

Author: H.A. Sharif
Year: 2026
"""

import os
import yaml
import copy
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
    robot_id = LaunchConfiguration('robot_id')
    config_name = LaunchConfiguration('config_name')
    config_path = PathJoinSubstitution([FindPackageShare('chroma_bringup'), 'config', 'profiles', config_name])

    args = [
        DeclareLaunchArgument('config_name', default_value='tb3_profile.yaml'),
        DeclareLaunchArgument('robot_id', default_value='tb3_0'),
        DeclareLaunchArgument('execution_interface', default_value='nav2'),
        DeclareLaunchArgument('telemetry_interface', default_value='standard'),
        DeclareLaunchArgument('initial_x', default_value=''),
        DeclareLaunchArgument('initial_y', default_value=''),
        DeclareLaunchArgument('initial_yaw', default_value=''),
    ]

    chroma_nodes = [
        # Core Nodes
        Node(
            package='chroma_core', executable='capability_manager',
            namespace=robot_id, name='capability_manager',
            parameters=[config_path, {'robot_id': robot_id}], output='screen'
        ),
        Node(
            package='chroma_core', executable='task_allocator',
            namespace=robot_id, name='task_allocator',
            parameters=[config_path, {'robot_id': robot_id}], output='screen'
        ),
        Node(
            package='chroma_core', executable='mission_executor',
            namespace=robot_id, name='mission_executor',
            parameters=[config_path, {'robot_id': robot_id}], output='screen'
        ),

        # Bridge Nodes
        Node(
            package='chroma_bridges', executable='nav2_bridge',
            namespace=robot_id, parameters=[{'robot_id': robot_id}],
            condition=LaunchConfigurationEquals('execution_interface', 'nav2')
        ),
        Node(
            package='chroma_bridges', executable='telemetry_bridge',
            namespace=robot_id, parameters=[config_path, {'robot_id': robot_id}],
            condition=LaunchConfigurationEquals('telemetry_interface', 'standard')
        ),
        Node(
            package='chroma_bridges', executable='degradation_manager',
            namespace=robot_id, parameters=[config_path, {'robot_id': robot_id}],
        ),
    ]

    nav2_stack = OpaqueFunction(function=launch_nav2)
    
    return LaunchDescription(args + chroma_nodes + [nav2_stack]) 

def launch_nav2(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('chroma_bringup')
    config_name = context.launch_configurations.get('config_name', 'tb3_profile.yaml')
    robot_id = context.launch_configurations.get('robot_id', 'tb3_0')

    with open(os.path.join(bringup_dir, 'config', 'profiles', config_name), 'r') as f:
        nav2_config = yaml.safe_load(f).get('/**', {}).get('ros__parameters', {}).get('nav2', {})

    if not nav2_config.get('enabled', False):
        return []

    # Prioritize launch arguments over profile defaults
    pose_config = nav2_config.get('initial_pose', {})
    init_x = float(context.launch_configurations.get('initial_x') or pose_config.get('x', 0.0))
    init_y = float(context.launch_configurations.get('initial_y') or pose_config.get('y', 0.0))
    init_yaw = float(context.launch_configurations.get('initial_yaw') or pose_config.get('yaw_degrees', 0.0))

    params_path = build_nav2_params(bringup_dir, nav2_config, init_x, init_y, init_yaw)
    map_path = build_map_yaml(bringup_dir, nav2_config)
    use_sim_time = str(nav2_config.get('use_sim_time', False)).lower()

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'nav2.launch.py')
            ),
            launch_arguments={
                'robot_id': robot_id,
                'map_yaml': map_path,
                'nav2_params': params_path,
                'use_sim_time': use_sim_time,
            }.items()
        )
    ]

# -------- Helpers --------

def build_nav2_params(bringup_dir, nav2_cfg, init_x, init_y, init_yaw):
    base_path = os.path.join(bringup_dir, 'config', 'nav2', 'nav2_base.yaml')
    override_name = nav2_cfg.get('params_file', 'nav2_tb3.yaml')
    override_path = os.path.join(bringup_dir, 'config', 'nav2', override_name)

    # Support robot specific nav2 overrides
    with open(base_path, 'r') as f_base, open(override_path, 'r') as f_over:
        merged = merge_dicts(yaml.safe_load(f_base), yaml.safe_load(f_over))

    # Nav2 ignores args for initial pose, set AMCL params directly
    amcl_params = merged.setdefault('amcl', {}).setdefault('ros__parameters', {})
    amcl_params['set_initial_pose'] = True
    amcl_params['initial_pose'] = {'x': init_x, 'y': init_y, 'yaw': init_yaw}

    return write_temp_yaml(merged, 'nav2_params_')

def merge_dicts(base, override):
    for key, value in override.items():
        if isinstance(value, dict) and key in base:
            merge_dicts(base[key], value)
        else:
            base[key] = copy.deepcopy(value)
    return base

def build_map_yaml(bringup_dir, nav2_cfg):
    maps_dir = os.path.join(bringup_dir, 'config', 'maps')
    map_name = nav2_cfg.get('map_yaml', 'empty_map.yaml')

    with open(os.path.join(maps_dir, map_name), 'r') as f:
        map_data = yaml.safe_load(f)

    # Nav2 requires absolute paths for map images
    map_data['image'] = os.path.join(maps_dir, map_data['image'])
    
    return write_temp_yaml(map_data, 'map_cfg_')

def write_temp_yaml(data, prefix):
    # Prevents race conditions during multi-robot spawn
    tmp = tempfile.NamedTemporaryFile(mode='w', prefix=prefix, suffix='.yaml', delete=False)
    yaml.dump(data, tmp)
    tmp.close()
    
    return tmp.name

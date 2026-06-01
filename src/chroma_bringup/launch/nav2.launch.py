import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    bringup_dir = get_package_share_directory('chroma_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    args = [
        DeclareLaunchArgument('robot_id', default_value='tb3_0'),
        DeclareLaunchArgument('map_yaml', default_value=''),
        DeclareLaunchArgument('nav2_params', default_value=os.path.join(bringup_dir, 'config', 'nav2', 'tb3_nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
    ]

    def make_patched_params(nav2_params, robot_id):
        odom_frame = f'{robot_id}/odom'
        base_frame = f'{robot_id}/base_link'
        base_footprint = f'{robot_id}/base_footprint'

        with open(nav2_params, 'r') as f:
            content = f.read()

        replacements = [
            ('ROBOT_NAMESPACE/base_footprint', base_footprint),
            ('ROBOT_NAMESPACE/base_link', base_frame),
            ('ROBOT_NAMESPACE/odom', odom_frame),
            ('ROBOT_NAMESPACE/scan', f'/{robot_id}/scan'),
            ('ROBOT_NAMESPACE', robot_id),
        ]
        for placeholder, value in replacements:
            content = content.replace(placeholder, value)

        tmp = tempfile.NamedTemporaryFile(
            mode='w',
            prefix=f'nav2_params_{robot_id}_',
            suffix='.yaml',
            delete=False
        )
        tmp.write(content)
        tmp.close()
        return tmp.name

    def launch_nav2(context, *args, **kwargs):
        robot_id = context.launch_configurations['robot_id']
        map_yaml = context.launch_configurations['map_yaml']
        nav2_params = context.launch_configurations['nav2_params']
        use_sim_time = context.launch_configurations['use_sim_time']
        initial_x = context.launch_configurations['initial_x']
        initial_y = context.launch_configurations['initial_y']
        initial_yaw = context.launch_configurations['initial_yaw']

        patched_params = make_patched_params(nav2_params, robot_id)

        nav2_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'namespace': robot_id,
                'use_namespace': 'True',
                'use_sim_time': use_sim_time,
                'params_file': patched_params,
                'map': map_yaml,
                'autostart': 'True',
                'use_composition': 'False',
                'set_initial_pose': 'True',
                'initial_pose.x': initial_x,
                'initial_pose.y': initial_y,
                'initial_pose.yaw': initial_yaw,
                'cmd_vel_topic': 'cmd_vel_raw', 
            }.items()
        )

        delayed_nav2_launch = TimerAction(
            period=6.0,
            actions=[nav2_bringup_launch]
        )

        return [delayed_nav2_launch]

    return LaunchDescription(args + [OpaqueFunction(function=launch_nav2)])
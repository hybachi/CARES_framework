import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    chroma_sim_dir = get_package_share_directory("chroma_simulation")
    chroma_bringup_dir = get_package_share_directory("chroma_bringup")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_file = os.path.join(chroma_sim_dir, "worlds", "usar_arena.sdf")
    swarm_config_path = os.path.join(chroma_sim_dir, "config", "swarm_config.yaml")
    ekf_config_file = os.path.join(chroma_sim_dir, 'config', 'ekf_config.yaml')

    set_env = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(os.path.dirname(chroma_sim_dir)),
    )

    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r -v1 {world_file}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    def spawn_robot(robot_namespace, model_name, x, y, z, yaw,
                    xacro_file, config_name,
                    execution_interface, telemetry_interface, params):

        robot_description = Command([
            "xacro ", xacro_file,
            " model_name:=", model_name,
        ])

        bridge_args = [
            f"/model/{model_name}/odom_raw@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            f"/model/{model_name}/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            f"/model/{model_name}/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            f"/model/{model_name}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
            f"/model/{model_name}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
        ]
        
        bridge_remaps = [
            "--ros-args",
            "-r", f"/model/{model_name}/odom_raw:=odom_raw",
            "-r", f"/model/{model_name}/scan:=scan",
            "-r", f"/model/{model_name}/imu:=imu",
            "-r", f"/model/{model_name}/joint_states:=joint_states",
            "-r", f"/model/{model_name}/tf:=tf",
            "-p", f"qos_overrides./{robot_namespace}/scan.publisher.reliability:=best_effort",
            "-p", f"qos_overrides./{robot_namespace}/scan.publisher.durability:=volatile",
            "-p", f"qos_overrides./{robot_namespace}/imu.publisher.reliability:=best_effort",
            "-p", f"qos_overrides./{robot_namespace}/imu.publisher.durability:=volatile",
        ]

        for b in params.get('custom_bridges', []):
            bridge_args.append(b.format(name=model_name))

        for r in params.get('custom_remaps', []):
            bridge_remaps.extend(["-r", r.format(name=model_name)])

        gazebo_bridge = Node(
            package="ros_ign_bridge",
            executable="parameter_bridge",
            namespace=robot_namespace,
            parameters=[{'use_sim_time': True}],
            output="screen",
            arguments=bridge_args + bridge_remaps,
        )

        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=robot_namespace,
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "frame_prefix": f"{robot_namespace}/",
                "robot_description": ParameterValue(robot_description, value_type=str),
            }],
            remappings=[
                ('/tf', f'/{robot_namespace}/tf'),
                ('/tf_static', f'/{robot_namespace}/tf_static'),
            ]
        )

        ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=robot_namespace,
            output='screen',
            parameters=[
                ekf_config_file,
                {
                    'use_sim_time': True,
                    'odom_frame': f'{robot_namespace}/odom',
                    'base_link_frame': f'{robot_namespace}/base_footprint',
                    'world_frame': f'{robot_namespace}/odom'
                }
            ],
            remappings=[
                ('odometry/filtered', 'odom'),
                ('/tf', f'/{robot_namespace}/tf'),
                ('/tf_static', f'/{robot_namespace}/tf_static'),
            ]
        )

        spawn = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-name", model_name,
                "-string", robot_description,
                "-x", x, "-y", y, "-z", z, "-Y", yaw,
            ],
        )

        chroma_brain = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(chroma_bringup_dir, 'launch', 'robot.launch.py')
            ),
            launch_arguments={
                'config_name': config_name,
                'robot_id': robot_namespace,
                'execution_interface': execution_interface,
                'telemetry_interface': telemetry_interface,
                'initial_x': x,
                'initial_y': y,
                'initial_yaw': yaw,
            }.items()
        )

        robot_nodes = [gazebo_bridge, robot_state_publisher, spawn, ekf_node, chroma_brain]

        for node_info in params.get('extra_nodes', []):
            robot_nodes.append(Node(
                package=node_info['package'],
                executable=node_info['executable'],
                namespace=robot_namespace,
                output="screen"
            ))

        return robot_nodes

    with open(swarm_config_path, 'r') as f:
        swarm_data = yaml.safe_load(f)

    spawn_actions = []
    for robot_id, params in swarm_data.get('swarm', {}).items():
        robot_nodes = spawn_robot(
            robot_namespace = robot_id,
            model_name = robot_id,
            x = str(params.get('x', 0.0)),
            y = str(params.get('y', 0.0)),
            z = str(params.get('z', 0.01)),
            yaw = str(params.get('yaw', 0.0)),
            xacro_file = os.path.join(chroma_sim_dir, "urdf", params['urdf']),
            config_name = params['config'],
            execution_interface = params.get('execution_interface',  'nav2'),
            telemetry_interface = params.get('telemetry_interface',  'standard'),
            params = params 
        )
        spawn_actions.extend(robot_nodes)
    
    global_clock = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="global_clock_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"
        ]
    )

    zone_monitor = Node(
        package="chroma_simulation",
        executable="zone_monitor.py",  
        name="zone_monitor",
        output="screen"
    )

    return LaunchDescription([
        set_env,
        launch_gazebo,
        global_clock,
        zone_monitor,
    ] + spawn_actions)
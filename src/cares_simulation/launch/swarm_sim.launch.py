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
    cares_sim_dir = get_package_share_directory("cares_simulation")
    cares_bringup_dir = get_package_share_directory("cares_bringup")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_file = os.path.join(cares_sim_dir, "worlds", "empty_world.sdf")
    swarm_config_path = os.path.join(cares_sim_dir, "config", "swarm_config.yaml")

    set_env = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(os.path.dirname(cares_sim_dir)),
    )

    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r -v4 {world_file}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    def spawn_robot(robot_namespace, model_name, x, y, z, yaw,
                    xacro_file, config_name,
                    execution_interface, telemetry_interface,
                    initial_pose):

        robot_description = Command([
            "xacro ", xacro_file,
            " model_name:=", model_name,
        ])

        gazebo_bridge = Node(
            package="ros_ign_bridge",
            executable="parameter_bridge",
            namespace=robot_namespace,
            parameters=[{'use_sim_time': True}],
            output="screen",
            arguments=[
                    f"/model/{model_name}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                    f"/model/{model_name}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                    f"/model/{model_name}/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                    f"/model/{model_name}/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
                    f"/model/{model_name}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
                    f"/model/{model_name}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                    "--ros-args",
                    "-r", f"/model/{model_name}/cmd_vel:=cmd_vel",
                    "-r", f"/model/{model_name}/odom:=odom",
                    "-r", f"/model/{model_name}/scan:=scan",
                    "-r", f"/model/{model_name}/imu:=imu",
                    "-r", f"/model/{model_name}/joint_states:=joint_states",
                    "-r", f"/model/{model_name}/tf:=tf",
                    # QoS overrides
                    "-p", f"qos_overrides./{robot_namespace}/scan.publisher.reliability:=best_effort",
                    "-p", f"qos_overrides./{robot_namespace}/scan.publisher.durability:=volatile",
                    "-p", f"qos_overrides./{robot_namespace}/imu.publisher.reliability:=best_effort",
                    "-p", f"qos_overrides./{robot_namespace}/imu.publisher.durability:=volatile",
                ],
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
                ('/tf',        f'/{robot_namespace}/tf'),
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

        cares_brain = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(cares_bringup_dir, 'launch', 'robot.launch.py')
            ),
            launch_arguments={
                'config_name': config_name,
                'robot_id': robot_namespace,
                'execution_interface': execution_interface,
                'telemetry_interface': telemetry_interface,
                'initial_x': str(initial_pose.get('x', 0.0)),
                'initial_y': str(initial_pose.get('y', 0.0)),
                'initial_yaw': str(initial_pose.get('yaw_degrees', 0.0)),
            }.items()
        )

        return [gazebo_bridge, robot_state_publisher, spawn, cares_brain]

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
            xacro_file = os.path.join(cares_sim_dir, "urdf", params['urdf']),
            config_name = params['config'],
            execution_interface = params.get('execution_interface',  'nav2'),
            telemetry_interface = params.get('telemetry_interface',  'standard'),
            initial_pose = params.get('initial_pose', {}),
        )
        spawn_actions.extend(robot_nodes)

    return LaunchDescription([
        set_env,
        launch_gazebo,
    ] + spawn_actions)
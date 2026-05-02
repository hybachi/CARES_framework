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

    # Paths
    world_file = os.path.join(cares_sim_dir, "worlds", "empty_world.sdf")
    swarm_config_path = os.path.join(cares_sim_dir, "config", "swarm_config.yaml")

    # Set Gazebo Environment Variable for Meshes
    set_env = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(os.path.dirname(cares_sim_dir)),
    )

    # Launch Gazebo
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r -v4 {world_file}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    def spawn_robot(robot_namespace, model_name, x, y, yaw, xacro_file, config_path):
        # Parse Xacro
        robot_description = Command([
            "xacro ", xacro_file,
            " model_name:=", model_name,
        ])

        # Gazebo -> ROS 2 Bridge
        gazebo_bridge = Node(
            package="ros_ign_bridge",
            executable="parameter_bridge",
            namespace=robot_namespace,
            output="screen",
            arguments=[
                f"/model/{model_name}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                f"/model/{model_name}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                f"/model/{model_name}/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                f"/model/{model_name}/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
                f"/model/{model_name}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
                "--ros-args",
                "-r", f"/model/{model_name}/cmd_vel:=cmd_vel",
                "-r", f"/model/{model_name}/odom:=odom",
                "-r", f"/model/{model_name}/scan:=scan",
                "-r", f"/model/{model_name}/imu:=imu",
                "-r", f"/model/{model_name}/joint_states:=joint_states",
            ],
        )

        # Robot State Publisher (TF Tree)
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
        )

        # Spawn in Gazebo
        spawn = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-name", model_name,
                "-string", robot_description,
                "-x", x,
                "-y", y,
                "-z", "0.01",
                "-Y", yaw,
            ],
        )

        # Launch the CARES Core for this specific robot
        cares_brain = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(cares_bringup_dir, 'launch', 'robot.launch.py')
            ),
            launch_arguments={
                'config_file': config_path,
                'robot_id': robot_namespace
            }.items()
        )

        return [gazebo_bridge, robot_state_publisher, spawn, cares_brain]

    # Read the YAML file dynamically
    with open(swarm_config_path, 'r') as file:
        swarm_data = yaml.safe_load(file)
    
    robots = swarm_data.get('swarm', {})
    spawn_actions = []

    # Loop through all robots in the YAML
    for robot_id, params in robots.items():
        config_path = os.path.join(cares_bringup_dir, 'config', params['config'])
        xacro_file = os.path.join(cares_sim_dir, "urdf", params['urdf'])

        x = str(params.get('x', 0.0))
        y = str(params.get('y', 0.0))
        yaw = str(params.get('yaw', 0.0))

        robot_nodes = spawn_robot(
            robot_namespace=robot_id,
            model_name=robot_id,
            x=x, y=y, yaw=yaw,
            xacro_file=xacro_file,
            config_path=config_path
        )
        
        spawn_actions.extend(robot_nodes)

    return LaunchDescription([
        set_env,
        launch_gazebo,
    ] + spawn_actions)
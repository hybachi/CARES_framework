"""
swarm_sim.launch.py
Deploys the Gazebo environment and dynamically spawns the robot swarm.

Author: H. A. Sharif
Year: 2026
"""

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
    sim_dir = get_package_share_directory("chroma_simulation")
    bringup_dir = get_package_share_directory("chroma_bringup")
    gz_dir = get_package_share_directory("ros_gz_sim")

    config_path = os.path.join(sim_dir, "config", "swarm_config.yaml")
    world_file = os.path.join(sim_dir, "worlds", "usar_arena.sdf")

    with open(config_path, 'r') as f:
        swarm_data = yaml.safe_load(f).get('swarm', {})

    base_nodes = [
        # Allow Gazebo to resolve nested model meshes
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(sim_dir)),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz_dir, "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": f"-r -v1 {world_file}", "on_exit_shutdown": "true"}.items(),
        ),
        
        # Synchronize ROS nodes with physics engine time
        Node(
            package="ros_ign_bridge", executable="parameter_bridge", 
            name="global_clock_bridge", arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"]
        ),
        
        Node(package="chroma_simulation", executable="zone_monitor.py", name="zone_monitor")
    ]

    robot_nodes = [
        node
        for robot_id, params in swarm_data.items()
        for node in build_robot(robot_id, params, sim_dir, bringup_dir)
    ]

    return LaunchDescription(base_nodes + robot_nodes)


def build_robot(robot_id, params, sim_dir, bringup_dir):
    x = str(params.get('x', 0.0))
    y = str(params.get('y', 0.0))
    z = str(params.get('z', 0.01))
    yaw = str(params.get('yaw', 0.0))

    xacro = os.path.join(sim_dir, "urdf", params['urdf'])
    desc = Command(["xacro ", xacro, " model_name:=", robot_id])

    bridge_args = [
        f"/model/{robot_id}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        f"/model/{robot_id}/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        f"/model/{robot_id}/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
        f"/model/{robot_id}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
        f"/model/{robot_id}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
    ] + [b.format(name=robot_id) for b in params.get('custom_bridges', [])]

    bridge_remaps = [
        "--ros-args",
        "-r", f"/model/{robot_id}/odom:=odom",
        "-r", f"/model/{robot_id}/scan:=scan",
        "-r", f"/model/{robot_id}/imu:=imu",
        "-r", f"/model/{robot_id}/joint_states:=joint_states",
        "-r", f"/model/{robot_id}/tf:=tf",
        "-p", f"qos_overrides./{robot_id}/scan.publisher.reliability:=best_effort",
        "-p", f"qos_overrides./{robot_id}/scan.publisher.durability:=volatile",
        "-p", f"qos_overrides./{robot_id}/imu.publisher.reliability:=best_effort",
        "-p", f"qos_overrides./{robot_id}/imu.publisher.durability:=volatile",
    ]
    
    for remap_rule in params.get('custom_remaps', []):
        bridge_remaps.extend(["-r", remap_rule.format(name=robot_id)])

    nodes = [
        Node(
            package="ros_ign_bridge", executable="parameter_bridge", namespace=robot_id,
            parameters=[{'use_sim_time': True}], arguments=bridge_args + bridge_remaps
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher", namespace=robot_id,
            parameters=[{
                "use_sim_time": True, 
                "frame_prefix": f"{robot_id}/", 
                "robot_description": ParameterValue(desc, value_type=str)
            }],
            # Ensure TF trees remain namespace isolated
            remappings=[('/tf', f'/{robot_id}/tf'), ('/tf_static', f'/{robot_id}/tf_static')]
        ),
        Node(
            package="ros_gz_sim", executable="create",
            arguments=["-name", robot_id, "-string", desc, "-x", x, "-y", y, "-z", z, "-Y", yaw]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'robot.launch.py')),
            launch_arguments={
                'config_name': params['config'],
                'robot_id': robot_id,
                'execution_interface': params.get('execution_interface', 'nav2'),
                'telemetry_interface': params.get('telemetry_interface', 'standard'),
                'initial_x': x,
                'initial_y': y,
                'initial_yaw': yaw,
            }.items()
        )
    ]

    extra_nodes = [
        Node(package=info['package'], executable=info['executable'], namespace=robot_id)
        for info in params.get('extra_nodes', [])
    ]

    return nodes + extra_nodes
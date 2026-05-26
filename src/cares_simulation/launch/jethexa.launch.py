import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory("cares_simulation")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    robot_namespace = "jethexa"     # ROS 2 context
    model_name = "jethexa"          # Gazebo context

    world_file = os.path.join(pkg_dir, "worlds", "empty_world.sdf")
    xacro_file = os.path.join(pkg_dir, "urdf", "jethexa", "robot.urdf.xacro")

    # Gazebo resource path
    set_env = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(os.path.dirname(pkg_dir)),
    )

    # Robot description (xacro)
    robot_description = Command([
        "xacro ", xacro_file,
        " model_name:=", model_name,
    ])

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=robot_namespace,
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "robot_description": ParameterValue(robot_description, value_type=str),
        }],
    )

    # Launch Ignition (Gazebo)
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # --- Gazebo to ROS bridge configuration ---
    # Base sensors and state
    bridge_args = [
        f"/model/{model_name}/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        f"/model/{model_name}/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
        f"/model/{model_name}/camera@sensor_msgs/msg/Image[ignition.msgs.Image",
        f"/model/{model_name}/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        f"/model/{model_name}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
    ]

    # Add 20 joint controllers for the legs and head
    joints = [
        "head_pan_joint", "head_tilt_joint",
        "coxa_joint_LF", "femur_joint_LF", "tibia_joint_LF",
        "coxa_joint_LM", "femur_joint_LM", "tibia_joint_LM",
        "coxa_joint_LR", "femur_joint_LR", "tibia_joint_LR",
        "coxa_joint_RF", "femur_joint_RF", "tibia_joint_RF",
        "coxa_joint_RM", "femur_joint_RM", "tibia_joint_RM",
        "coxa_joint_RR", "femur_joint_RR", "tibia_joint_RR"
    ]

    # Map Ignition Double to std_msgs Float64 for position commands
    for joint in joints:
        bridge_args.append(f"/model/{model_name}/joint/{joint}/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double")

    # Add ROS topic remaps
    bridge_args.extend([
        "--ros-args",
        "-r", f"/model/{model_name}/scan:=scan",
        "-r", f"/model/{model_name}/imu:=imu",
        "-r", f"/model/{model_name}/camera:=camera/image_raw",
        "-r", f"/model/{model_name}/camera_info:=camera/camera_info",
        "-r", f"/model/{model_name}/joint_states:=joint_states",
    ])

    for joint in joints:
        bridge_args.extend(["-r", f"/model/{model_name}/joint/{joint}/cmd_pos:=joint/{joint}/cmd_pos"])


    # Ignition (Gazebo) to ROS bridge Node
    gazebo_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        namespace=robot_namespace,
        output="screen",
        arguments=bridge_args,
    )

    # Spawn the robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", model_name,
            "-string", robot_description,
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.3",
            "-Y", "0.0",
        ],
    )

    return LaunchDescription([
        set_env,
        launch_gazebo,
        gazebo_bridge,
        robot_state_publisher,
        spawn_robot,
    ])
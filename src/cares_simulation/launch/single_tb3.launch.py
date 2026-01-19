import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory("robot_description")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Default robot
    robot_namespace = "tb3"     # ROS2 context
    model_name = "tb3"          # Gazebo context

    # Paths
    world_file = os.path.join(pkg_dir, "worlds", "empty_world.sdf")
    xacro_file = os.path.join(pkg_dir, "urdf", "turtlebot3_burger", "robot.urdf.xacro")

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

    # Launch Ignition (Gazebo) Fortress
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Ignition (Gazebo) to ROS bridge
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
            "-z", "0.01",
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

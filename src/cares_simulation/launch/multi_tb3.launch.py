import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory("robot_description")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Paths
    world_file = os.path.join(pkg_dir, "worlds", "empty_world.sdf")
    xacro_file = os.path.join(pkg_dir, "urdf", "turtlebot3_burger", "robot.urdf.xacro")

    # Launch arguments
    tb3_0_x = LaunchConfiguration("tb3_0_x")
    tb3_0_y = LaunchConfiguration("tb3_0_y")
    tb3_0_yaw = LaunchConfiguration("tb3_0_yaw")

    tb3_1_x = LaunchConfiguration("tb3_1_x")
    tb3_1_y = LaunchConfiguration("tb3_1_y")
    tb3_1_yaw = LaunchConfiguration("tb3_1_yaw")

    tb3_2_x = LaunchConfiguration("tb3_2_x")
    tb3_2_y = LaunchConfiguration("tb3_2_y")
    tb3_2_yaw = LaunchConfiguration("tb3_2_yaw")

    # Set Gazebo Environment Variable
    set_env = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(os.path.dirname(pkg_dir)),
    )

    # Launch Gazebo
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Function for spawning robots
    def spawn_robot(robot_namespace, model_name, x, y, yaw):
        robot_description = Command([
            "xacro ", xacro_file,
            " model_name:=", model_name,
        ])

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

        return [gazebo_bridge, robot_state_publisher, spawn]

    return LaunchDescription([
        set_env,
        launch_gazebo,

        # Declare pose
        DeclareLaunchArgument("tb3_0_x", default_value="0.0"),
        DeclareLaunchArgument("tb3_0_y", default_value="0.0"),
        DeclareLaunchArgument("tb3_0_yaw", default_value="0.0"),

        DeclareLaunchArgument("tb3_1_x", default_value="1.0"),
        DeclareLaunchArgument("tb3_1_y", default_value="0.0"),
        DeclareLaunchArgument("tb3_1_yaw", default_value="0.0"),

        DeclareLaunchArgument("tb3_2_x", default_value="0.0"),
        DeclareLaunchArgument("tb3_2_y", default_value="1.0"),
        DeclareLaunchArgument("tb3_2_yaw", default_value="0.0"),

        # Spawn Robots
        *spawn_robot("tb3_0", "tb3_0", tb3_0_x, tb3_0_y, tb3_0_yaw),
        *spawn_robot("tb3_1", "tb3_1", tb3_1_x, tb3_1_y, tb3_1_yaw),
        *spawn_robot("tb3_2", "tb3_2", tb3_2_x, tb3_2_y, tb3_2_yaw),
    ])

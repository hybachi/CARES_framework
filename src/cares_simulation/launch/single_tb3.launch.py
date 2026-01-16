import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution

from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('robot_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_namespace = LaunchConfiguration('robot_namespace', default='tb3')

    # New: model name in Gazebo (avoid slashes; keep it simple like tb3_0)
    model_name = LaunchConfiguration('model_name', default=robot_namespace)

    # New: TF frame prefix (prevents TF collisions in multi-robot)
    # Default: "<robot_namespace>/"
    frame_prefix = LaunchConfiguration(
        'frame_prefix',
        default=[robot_namespace, TextSubstitution(text='/')]
    )

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.01')
    yaw = LaunchConfiguration('yaw', default='0.0')

    # Paths
    world_file = os.path.join(pkg_dir, 'worlds', 'empty_world.sdf')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'turtlebot3_burger', 'robot.urdf.xacro')
    bridge_params = os.path.join(pkg_dir, 'config', 'tb3_bridge.yaml')

    # Set Gazebo resource path
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(os.path.dirname(pkg_dir))
    )

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='tb3',
        description='Robot namespace (e.g. tb3_0, tb3_1). Used for ROS namespace.'
    )

    declare_model_name = DeclareLaunchArgument(
        'model_name',
        default_value=robot_namespace,
        description='Gazebo model name (should not contain slashes). Defaults to robot_namespace.'
    )

    declare_frame_prefix = DeclareLaunchArgument(
        'frame_prefix',
        default_value=[robot_namespace, TextSubstitution(text='/')],
        description="TF frame prefix to avoid collisions (e.g. 'tb3_0/' or 'tb3_0_')."
    )

    declare_x_pose = DeclareLaunchArgument(
        'x_pose', default_value='0.0', description='X position of the robot'
    )
    declare_y_pose = DeclareLaunchArgument(
        'y_pose', default_value='0.0', description='Y position of the robot'
    )
    declare_z_pose = DeclareLaunchArgument(
        'z_pose', default_value='0.01', description='Z position of the robot'
    )
    declare_yaw = DeclareLaunchArgument(
        'yaw', default_value='0.0', description='Yaw orientation of the robot'
    )

    # Process xacro file
    # NOTE: we keep passing args if your xacro expects them, but your new xacro
    # should not bake namespaces into link/joint names.
    robot_description_xacro = Command([
        'xacro ', xacro_file,
        ' robot_namespace:=', robot_namespace,
        ' use_sim_time:=', use_sim_time
    ])

    # Robot state publisher (namespaced) + frame prefix (critical for multi-robot TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_prefix': frame_prefix,  # <-- key change
            'robot_description': ParameterValue(robot_description_xacro, value_type=str),
        }]
    )

    # Launch Gazebo Server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v4 ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', model_name,              # <-- safer than using namespace directly
            '-string', robot_description_xacro,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw,
        ],
        output='screen'
    )

    # Bridge between Gazebo and ROS (runs in the robot namespace)
    # With your updated URDF using RELATIVE topics, this is what you want.
    ros_gz_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        namespace=robot_namespace,
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # Create launch description
    ld = LaunchDescription()

    ld.add_action(set_env_vars_resources)

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_robot_namespace)
    ld.add_action(declare_model_name)
    ld.add_action(declare_frame_prefix)

    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(declare_z_pose)
    ld.add_action(declare_yaw)

    ld.add_action(gzserver_cmd)
    ld.add_action(ros_gz_bridge)

    ld.add_action(robot_state_publisher)
    # ld.add_action(diff_drive_spawner)
    # ld.add_action(joint_broad_spawner)
    ld.add_action(spawn_robot)

    return ld

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cares_sim_dir = get_package_share_directory('cares_simulation')
    swarm_config_path = os.path.join(cares_sim_dir, 'config', 'swarm_config.yaml')

    with open(swarm_config_path, 'r') as f:
        swarm_data = yaml.safe_load(f)

    relay_nodes = []
    for robot_id in swarm_data.get('swarm', {}).keys():
        # Relay /tb3_0/tf → /tf
        relay_nodes.append(Node(
            package='topic_tools',
            executable='relay',
            name=f'tf_relay_{robot_id}',
            arguments=[f'/{robot_id}/tf', '/tf'],
            output='screen'
        ))
        # Relay /tb3_0/tf_static → /tf_static
        relay_nodes.append(Node(
            package='topic_tools',
            executable='relay',
            name=f'tf_static_relay_{robot_id}',
            arguments=[f'/{robot_id}/tf_static', '/tf_static'],
            output='screen'
        ))

    return LaunchDescription(relay_nodes)
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the list of robots
    robots = ['tb3_0', 'tb3_1', 'tb3_2']
    launch_actions = []

    for robot_name in robots:
        node = Node(
            package='cares_core',
            executable='capability_manager',
            namespace=robot_name,      
            name='capability_manager',    
            output='screen',
            parameters=[
                {'robot_id': robot_name} 
            ]
        )
        launch_actions.append(node)

        node = Node(
            package='cares_core',
            executable='task_allocator',
            namespace=robot_name,      
            name='task_allocator',    
            output='screen',
            parameters=[
                {'robot_id': robot_name} 
            ]
        )
        launch_actions.append(node)

    return LaunchDescription(launch_actions)
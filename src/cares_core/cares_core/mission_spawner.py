import rclpy
from rclpy.node import Node
from cares_interfaces.msg import Task
from geometry_msgs.msg import Point

class MissionSpawner(Node):
    def __init__(self):
        super().__init__('mission_spawner')
        self.pub = self.create_publisher(Task, '/mission/tasks', 10)
        self.timer = self.create_timer(5.0, self.publish_tasks)

    def publish_tasks(self):
        # Task 1: Needs Vision
        t1 = Task()
        t1.task_id = "task_aerial_scan"
        t1.type = "SCAN"
        t1.priority = 1.0
        t1.required_capabilities = ["VISION"]
        t1.min_capability_score = 0.8
        t1.location = Point(x=5.0, y=5.0, z=2.0)
        self.pub.publish(t1)

        # Task 2: Needs Mobility
        t2 = Task()
        t2.task_id = "task_rubble_cross"
        t2.type = "SEARCH"
        t2.priority = 0.8
        t2.required_capabilities = ["MOBILITY"]
        t2.min_capability_score = 0.7
        t2.location = Point(x=10.0, y=0.0, z=0.0)
        self.pub.publish(t2)

        # Task 3: Less Mobility
        t3 = Task()
        t3.task_id = "task_move"
        t3.type = "SEARCH"
        t3.priority = 0.6
        t3.required_capabilities = ["MOBILITY"]
        t3.min_capability_score = 0.4
        t3.location = Point(x=5.0, y=0.3, z=0.0)
        self.pub.publish(t3)
        
        self.get_logger().info("Published Mission Tasks")
        self.timer.cancel() # Publish only once

def main():
    rclpy.init()
    node = MissionSpawner()
    rclpy.spin(node)
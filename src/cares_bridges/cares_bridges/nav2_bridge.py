import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from cares_interfaces.msg import Task, TaskAllocation

class Nav2Bridge(Node):
    def __init__(self):
        super().__init__('nav2_bridge')
        self.declare_parameter('robot_id', 'unknown')
        self.robot_id = self.get_parameter('robot_id').value

        self.active_task = None

        self.create_subscription(Task, 'execute_task', self.task_cb, 10)
        self.create_subscription(TaskAllocation, '/swarm/allocation', self.alloc_cb, 10)
        self.alloc_pub = self.create_publisher(TaskAllocation, '/swarm/allocation', 10)

        # Connect to Nav2 in the robot's namespace
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def task_cb(self, msg):
        if self.active_task: return
        self.active_task = msg
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = msg.location.x
        goal.pose.pose.position.y = msg.location.y
        goal.pose.pose.orientation.w = 1.0 

        self.send_goal_future = self.nav_client.send_goal_async(goal)
        self.send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.report_status("FAILED")
            return
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_cb)

    def get_result_cb(self, future):
        status = future.result().status
        if status == 4: # SUCCEEDED
            self.report_status("COMPLETED")

    def alloc_cb(self, msg):
        if self.active_task and msg.task_id == self.active_task.task_id:
            if msg.status == "ABORTED" and msg.robot_id == self.robot_id:
                if hasattr(self, 'send_goal_future') and self.send_goal_future.result():
                    self.nav_client._cancel_goal_async(self.send_goal_future.result())
                self.active_task = None

    def report_status(self, status):
        msg = TaskAllocation(task_id=self.active_task.task_id, robot_id=self.robot_id, status=status)
        self.alloc_pub.publish(msg)
        self.active_task = None

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Nav2Bridge())
    rclpy.shutdown()
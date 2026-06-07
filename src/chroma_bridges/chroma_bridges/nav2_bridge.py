import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from chroma_interfaces.msg import Task, TaskAllocation

class Nav2Bridge(Node):
    def __init__(self):
        super().__init__('nav2_bridge')
        self.declare_parameter('robot_id', 'unknown')
        self.robot_id = self.get_parameter('robot_id').value

        self.active_task = None
        self.goal_handle = None 

        self.create_subscription(Task, 'execute_waypoint', self.task_cb, 10)
        self.create_subscription(TaskAllocation, '/swarm/allocation', self.alloc_cb, 10)
        self.alloc_pub = self.create_publisher(TaskAllocation, 'execution_status', 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def task_cb(self, msg):
        if self.active_task:
            return

        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Nav2 action server not available")
            return

        self.active_task = msg
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()  
        
        # Position
        goal.pose.pose.position.x = msg.location.x
        goal.pose.pose.position.y = msg.location.y
        
        # Orientation
        yaw = msg.location.z
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f"Navigating to ({msg.location.x:.2f}, {msg.location.y:.2f}) facing {math.degrees(yaw):.0f} deg"
        )
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self.report_status("FAILED")
            return
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_cb)

    def get_result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.report_status("COMPLETED")
        else:
            self.get_logger().warn(f"Navigation ended with status {status}")
            self.report_status("FAILED")

    def alloc_cb(self, msg):
        if (self.active_task
                and msg.task_id  == self.active_task.task_id
                and msg.robot_id == self.robot_id
                and msg.status   == "ABORTED"):
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            self.active_task = None

    def report_status(self, status):
        msg = TaskAllocation()
        msg.task_id = self.active_task.task_id
        msg.robot_id = self.robot_id
        msg.status = status
        self.alloc_pub.publish(msg)
        self.active_task = None
        self.goal_handle = None

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Nav2Bridge())
    rclpy.shutdown()
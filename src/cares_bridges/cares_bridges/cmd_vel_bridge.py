import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from cares_interfaces.msg import Task, TaskAllocation

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.declare_parameter('robot_id', 'unknown')
        self.robot_id = self.get_parameter('robot_id').value

        self.active_task = None
        self.current_x, self.current_y, self.current_yaw = 0.0, 0.0, 0.0
        self.obstacle_in_front = False

        self.create_subscription(Task, 'execute_task', self.task_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)
        self.create_subscription(TaskAllocation, '/swarm/allocation', self.alloc_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.alloc_pub = self.create_publisher(TaskAllocation, '/swarm/allocation', 10)

        self.create_timer(0.1, self.control_loop) # 10Hz
        self.get_logger().info(f"CmdVel Sim Bridge Active for {self.robot_id}")

    def task_cb(self, msg):
        if self.active_task is None:
            self.active_task = msg

    def alloc_cb(self, msg):
        if self.active_task and msg.task_id == self.active_task.task_id:
            if msg.status == "ABORTED" and msg.robot_id == self.robot_id:
                self.cmd_pub.publish(Twist()) # Stop immediately
                self.active_task = None

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def scan_cb(self, msg):
        # Check front 30 degrees for obstacles under 0.4 meters
        cone = msg.ranges[:15] + msg.ranges[-15:]
        valid = [r for r in cone if 0.1 < r < 10.0]
        self.obstacle_in_front = len(valid) > 0 and min(valid) < 0.4

    def control_loop(self):
        if not self.active_task: return
        cmd = Twist()

        dx = self.active_task.location.x - self.current_x
        dy = self.active_task.location.y - self.current_y
        dist = math.hypot(dx, dy)

        if dist < 0.15:
            self.cmd_pub.publish(cmd) # Stop
            alloc = TaskAllocation(task_id=self.active_task.task_id, robot_id=self.robot_id, status="COMPLETED")
            self.alloc_pub.publish(alloc)
            self.active_task = None
            return

        # Obstacle Subsumption
        if self.obstacle_in_front:
            cmd.angular.z = 1.0 # Turn left to avoid
            self.cmd_pub.publish(cmd)
            return

        # Go-To-Goal
        target_yaw = math.atan2(dy, dx)
        err = math.atan2(math.sin(target_yaw - self.current_yaw), math.cos(target_yaw - self.current_yaw))

        if abs(err) > 0.3:
            cmd.angular.z = 1.0 if err > 0 else -1.0
        else:
            cmd.linear.x = min(0.3, dist)
            cmd.angular.z = 1.5 * err

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CmdVelBridge())
    rclpy.shutdown()
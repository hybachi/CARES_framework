import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from chroma_interfaces.msg import SwarmStatus
import random

class HardwareDegradationManager(Node):
    def __init__(self):
        super().__init__('degradation_manager')
        
        self.declare_parameter('robot_id', 'unknown')
        self.robot_id = self.get_parameter('robot_id').value

        self.caps = {
            "MOBILITY": 1.0,
            "VISION": 1.0,
            # TODO: handle battery degradation
            # "BATTERY": 1.0
        }

        self.last_cmd = Twist()

        self.create_subscription(SwarmStatus, '/swarm/status', self.status_cb, 10)

        self.create_subscription(Twist, 'cmd_vel_raw', self.cmd_cb, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # TODO: implement vision degradation
        # self.create_subscription(LaserScan, 'scan_raw', self.scan_cb, 10)
        # self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        self.get_logger().info(f"[{self.robot_id}] Hardware Degradation Manager Active.")

    def status_cb(self, msg: SwarmStatus):
        if msg.robot_id == self.robot_id:
            for cap in msg.capabilities:
                if cap.type in self.caps and self.caps[cap.type] != cap.value:
                    self.caps[cap.type] = cap.value
                    self.get_logger().info(f"{cap.type} degraded to: {cap.value:.2f}")

                    if cap.type == "MOBILITY":
                        self.cmd_cb(self.last_cmd)


    def cmd_cb(self, msg: Twist):
        mobility_score = self.caps["MOBILITY"]

        self.last_cmd = msg 
        
        throttled_msg = Twist()
        throttled_msg.linear.x = msg.linear.x * mobility_score
        throttled_msg.linear.y = msg.linear.y * mobility_score
        throttled_msg.angular.z = msg.angular.z * mobility_score
        
        self.vel_pub.publish(throttled_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(HardwareDegradationManager())
    rclpy.shutdown()
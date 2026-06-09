"""
degradation_manager.py
Simulates hardware degradation by scaling physical commands and sensor data.

Author: H.A. Sharif
Year: 2026
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from chroma_interfaces.msg import SwarmStatus

class DegradationManager(Node):
    def __init__(self):
        super().__init__('degradation_manager')
        self.setup_params()
        self.setup_state()
        self.setup_pubs_subs()
        self.get_logger().info(f"[{self.robot_id}] Degradation Manager Active")

    # -------- Setup --------

    def setup_params(self):
        self.declare_parameter('robot_id', 'unknown')
        self.robot_id = self.get_parameter('robot_id').value

    def setup_state(self):
        self.caps = {
            "MOBILITY": 1.0,
            "VISION": 1.0,
            # TODO: handle battery degradation
            # "BATTERY": 1.0
        }

        self.last_cmd_vel = Twist()

    def setup_pubs_subs(self):
        self.create_subscription(SwarmStatus, '/swarm/status', self.status_cb, 10)
        self.create_subscription(Twist, 'cmd_vel_raw', self.cmd_vel_cb, 10)

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # TODO: implement vision degradation
        # self.create_subscription(LaserScan, 'scan_raw', self.scan_cb, 10)
        # self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

    # -------- Callbacks --------

    def status_cb(self, msg):
        if msg.robot_id != self.robot_id:
            return
        
        for cap in msg.capabilities:
            if cap.type in self.caps and self.caps[cap.type] != cap.value:
                self.caps[cap.type] = cap.value
                self.get_logger().info(f"{cap.type} changed to: {cap.value:.2f}")

                # Bypass capability update delay
                if cap.type == "MOBILITY":
                    self.cmd_vel_cb(self.last_cmd_vel)

    def cmd_vel_cb(self, msg):
        self.last_cmd_vel = msg
        mobility_score = self.caps["MOBILITY"]

        throttled_msg = Twist()
        throttled_msg.linear.x = msg.linear.x * mobility_score
        throttled_msg.linear.y = msg.linear.y * mobility_score
        throttled_msg.angular.z = msg.angular.z * mobility_score
        
        self.vel_pub.publish(throttled_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DegradationManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
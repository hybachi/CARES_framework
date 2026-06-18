"""
degradation_manager.py
Simulates hardware degradation by scaling physical commands and sensor data.

Author: H.A. Sharif
Year: 2026
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from chroma_interfaces.msg import SwarmStatus
import random

class DegradationManager(Node):
    def __init__(self):
        super().__init__(
            'degradation_manager',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        self.setup_params()
        self.setup_state()
        self.setup_pubs_subs()
        self.get_logger().info(f"[{self.robot_id}] Degradation Manager Active")

    # -------- Setup --------

    def setup_params(self):
        self.robot_id = self.get_parameter('robot_id').value if self.has_parameter('robot_id') else 'unknown'

        yaml_caps = {k: v.value for k, v in self.get_parameters_by_prefix('capabilities').items()}
        
        self.baselines = {
            "MOBILITY": yaml_caps.get("MOBILITY", 1.0),
            "PERCEPTION": yaml_caps.get("PERCEPTION", 1.0)
        }

    def setup_state(self):
        self.caps = self.baselines.copy()
        self.last_cmd_vel = Twist()

    def setup_pubs_subs(self):
        self.create_subscription(SwarmStatus, '/swarm/status', self.status_cb, 10)

        self.create_subscription(Twist, 'cmd_vel_raw', self.cmd_vel_cb, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_subscription(LaserScan, 'scan_raw', self.scan_cb, qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)

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

        baseline = self.baselines.get("MOBILITY", 1.0)
        current = self.caps.get("MOBILITY", 1.0)
        ratio = (current / baseline) if baseline > 0 else 0.0

        throttled_msg = Twist()
        throttled_msg.linear.x = msg.linear.x * ratio
        throttled_msg.linear.y = msg.linear.y * ratio
        throttled_msg.angular.z = msg.angular.z * ratio
        
        self.vel_pub.publish(throttled_msg)

    def scan_cb(self, msg):
        baseline = self.baselines.get("PERCEPTION", 1.0)
        current = self.caps.get("PERCEPTION", 1.0)
        ratio = (current / baseline) if baseline > 0 else 0.0

        if ratio >= 0.99 or baseline == 0.0:
            self.scan_pub.publish(msg)
            return

        # Additive White Gaussian Noise (AWGN)
        noise_variance = (1.0 - ratio) * 0.5
        
        noisy_ranges = []
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                noisy_r = r + random.gauss(0.0, noise_variance)

                # Ensure the noise doesn't violate sensor bounds
                noisy_r = max(msg.range_min, min(noisy_r, msg.range_max))
                noisy_ranges.append(noisy_r)
            else:
                noisy_ranges.append(r)

        degraded_msg = LaserScan(
            header=msg.header,
            angle_min=msg.angle_min,
            angle_max=msg.angle_max,
            angle_increment=msg.angle_increment,
            time_increment=msg.time_increment,
            scan_time=msg.scan_time,
            range_min=msg.range_min,
            range_max=msg.range_max,
            intensities=msg.intensities,
            ranges=noisy_ranges
        )
        
        self.scan_pub.publish(degraded_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DegradationManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
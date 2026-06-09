"""
capability_manager.py
Maintains and publishes the robot's hardware capability health scores.

Author: H.A. Sharif
Year: 2026
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from chroma_interfaces.msg import SwarmStatus, Capability

class CapabilityManager(Node):
    def __init__(self):
        super().__init__(
            'capability_manager',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        self.setup_params()
        self.setup_state()
        self.setup_pubs_subs()
        self.get_logger().info(f"[{self.robot_id}] Tracking: {list(self.caps.keys())}")

    # -------- Setup --------

    def setup_params(self):
        self.robot_id = self.get_parameter('robot_id').value if self.has_parameter('robot_id') else 'unknown'
        self.robot_type = self.get_parameter('robot_type').value if self.has_parameter('robot_type') else 'UGV'

    def setup_state(self):
        self.fault_map = {}
        self.caps = {k: v.value for k, v in self.get_parameters_by_prefix('capabilities').items()}
        self.thresholds = {k: v.value for k, v in self.get_parameters_by_prefix('thresholds').items()}
        self.fault_impacts = {k: v.value for k, v in self.get_parameters_by_prefix('fault_impacts').items()}

    def setup_pubs_subs(self):
        self.create_subscription(Capability, 'telemetry', self.telemetry_cb, 10)
        self.create_subscription(String, 'inject_fault', self.fault_cb, 10)
        
        swarm_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.swarm_pub = self.create_publisher(SwarmStatus, '/swarm/status', swarm_qos)
        
        self.create_timer(5.0, self.publish_state) 

    # -------- Callbacks --------

    def telemetry_cb(self, msg):
        self.caps[msg.type] = msg.value

    def fault_cb(self, msg):
        fault = msg.data
        if fault == "reset":
            self.fault_map.clear()
            self.get_logger().info("All faults reset")
        else:
            self.fault_map[fault] = True
            self.get_logger().warn(f"{fault} injected")
        
        self.publish_state()

    # -------- Logic --------

    def publish_state(self):
        msg = SwarmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id
        msg.robot_type = self.robot_type

        for cap_type, base_val in self.caps.items():
            c = Capability()
            c.type = cap_type
            
            fail_key = f"{cap_type.lower()}_failure"
            penalty_key = f"{cap_type.lower()}_penalty"
            
            impact = 1.0
            if fail_key in self.fault_map:
                impact = self.fault_impacts.get(fail_key, 0.0)
            elif penalty_key in self.fault_map:
                impact = self.fault_impacts.get(penalty_key, 0.8)
                
            c.value = float(base_val * impact)
            c.is_degraded = c.value < self.thresholds.get(cap_type, 0.2)
            msg.capabilities.append(c)

        self.swarm_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CapabilityManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
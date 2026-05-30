import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from chroma_interfaces.msg import SwarmStatus, Capability

class CapabilityManager(Node):
    def __init__(self):
        super().__init__('capability_manager')
        
        # Parameters
        self.declare_parameter('robot_id', 'unknown')
        self.declare_parameter('robot_type', 'UGV')

        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value

        # State Dictionaries
        self.caps = {}
        self.thresholds = {}
        self.fault_map = {}
        self.fault_impacts = {}

        # Load Parameters from YAML
        core_capabilities = ['MOBILITY', 'VISION', 'NETWORK', 'BATTERY']
        for cap in core_capabilities:
            # Declare
            self.declare_parameter(f'capabilities.{cap}', 1.0)
            self.declare_parameter(f'thresholds.{cap}', 0.2)
            self.declare_parameter(f'fault_impacts.{cap.lower()}_failure', 0.1)
            
            # Load into dicts
            self.caps[cap] = self.get_parameter(f'capabilities.{cap}').value
            self.thresholds[cap] = self.get_parameter(f'thresholds.{cap}').value
            self.fault_impacts[f'{cap.lower()}_failure'] = self.get_parameter(f'fault_impacts.{cap.lower()}_failure').value

        # Subscribers
        self.create_subscription(Capability, 'telemetry', self.telemetry_callback, 10)
        self.create_subscription(String, 'inject_fault', self.fault_callback, 10)
        
        # QoS profile
        swarm_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
            # TODO: deadline property to handle missed messages
        )

        # Publishes to a global topic, but listeners filter by ID
        self.swarm_pub = self.create_publisher(SwarmStatus, '/swarm/status', swarm_qos)
        self.create_timer(5.0, self.publish_status) # Publish once every 5 seconds


    def telemetry_callback(self, msg):
        self.caps[msg.type] = msg.value

    def fault_callback(self, msg):
        fault = msg.data
        if fault == "reset":
            self.fault_map.clear()
            self.get_logger().info("All faults reset.")
        else:
            self.fault_map[fault] = True
            self.get_logger().warn(f"Fault injected: {fault}")

    def publish_status(self):
        msg = SwarmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id
        msg.robot_type = self.robot_type

        for cap_type, base_val in self.caps.items():
            c = Capability()
            c.type = cap_type
            
            fault_key = f"{cap_type.lower()}_failure"
            if fault_key in self.fault_map:
                impact = self.fault_impacts.get(fault_key, 0.1)
                c.value = float(base_val * impact)
            else:
                c.value = float(base_val)
                
            c.is_degraded = c.value < self.thresholds.get(cap_type, 0.2)
            msg.capabilities.append(c)

        self.swarm_pub.publish(msg)
    
    
def main(args=None):
    rclpy.init(args=args)
    node = CapabilityManager()
    rclpy.spin(node)
    rclpy.shutdown()
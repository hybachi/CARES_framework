import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cares_interfaces.msg import SwarmStatus, Capability

class CapabilityManager(Node):
    def __init__(self):
        super().__init__('capability_manager')
        
        # Parameters
        self.declare_parameter('robot_id', 'unknown')
        self.declare_parameter('robot_type', 'UGV')

        self.robot_id = self.get_parameter('robot_id').value
        self.robot_type = self.get_parameter('robot_type').value

        self.caps = {}
        self.thresholds = {}
        cap_types = ['MOBILITY', 'VISION', 'NETWORK', 'BATTERY']

        for cap in cap_types:
            self.declare_parameter(f'capabilities.{cap}', 0.0)
            self.declare_parameter(f'thresholds.{cap}', 0.2)

            self.caps[cap] = self.get_parameter(f'capabilities.{cap}').value
            self.thresholds[cap] = self.get_parameter(f'thresholds.{cap}').value

        
        # State variables
        self.current_speed = 0.0
        self.target_speed = 0.0
        self.battery_level = 1.0  # Simulating 100%
        self.fault_map = {}       # Stores active faults
        self.swarm_peers = {}     # Stores robot peers

        # Subscribers
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
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

        # Subscriber for peer discovery
        self.create_subscription(SwarmStatus, '/swarm/status', self.peer_callback, swarm_qos)


    def cmd_callback(self, msg):
        # Robot mobility expectation
        self.target_speed = msg.linear.x


    def odom_callback(self, msg):
        # Robot mobility reality
        self.current_speed = msg.twist.twist.linear.x


    def fault_callback(self, msg):
        # "mobility_failure" or "reset"
        # TODO: add network failure and battery failure
        fault = msg.data
        if fault == "reset":
            self.fault_map.clear()
        else:
            self.fault_map[fault] = True
        self.get_logger().warn(f"Fault State Changed: {self.fault_map}")


    def peer_callback(self, msg):
        # Ignore self
        if msg.robot_id == self.robot_id:
            return
        
        # Store peers
        self.swarm_peers[msg.robot_id] = {
            'data': msg,
            'last_seen': self.get_clock().now()
        }

        # TODO: Debug print, remove later
        # self.get_logger().info(f"Discovered {len(self.swarm_peers)} neighbours")


    def calculate_mobility(self):
        mobility_score = 1.0
        
        # Apply degradation logic (fault injection)
        if "mobility_failure" in self.fault_map:
            mobility_score = 0.1
        
        # If target is high but actual is low, efficiency drops
        if self.target_speed > 0.1:
            efficiency = self.current_speed / self.target_speed
            # Clamp to 0-1
            efficiency = max(0.0, min(1.0, efficiency))
            # 70% based on faults, 30% based on physics
            mobility_score = (mobility_score * 0.7) + (efficiency * 0.3)
            
        return float(mobility_score)
    

    def calculate_network_health(self):
        # Remove peers with stale data (more than 5 seconds)
        current_time = self.get_clock().now()
        timeout = Duration(seconds=5.0)

        dead_peers = []
        for peer_id, peer in self.swarm_peers.items():
            if (current_time - peer['last_seen']) > timeout:
                dead_peers.append(peer_id)

        for peer_id in dead_peers:
            del self.swarm_peers[peer_id]
            self.get_logger().warn(f"Lost peer {peer_id}")

        # Network health based on isolation
        peer_count = len(self.swarm_peers)
        if peer_count == 0:
            return 0.1
        else:
            return 1.0


    def publish_status(self):
        msg = SwarmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id
        msg.robot_type = self.robot_type

        self.caps['BATTERY'] = max(0.0, self.caps['BATTERY'] - 0.001)

        for cap, current_val in self.caps.items():
            c = Capability()
            c.type = cap
            c.value = current_val
            c.is_degraded = c.value < self.thresholds.get(cap, 0.2)
            msg.capabilities.append(c)

        self.swarm_pub.publish(msg)
    
    
def main(args=None):
    rclpy.init(args=args)
    node = CapabilityManager()
    rclpy.spin(node)
    rclpy.shutdown()
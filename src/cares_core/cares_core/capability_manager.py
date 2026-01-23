import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cares_interfaces.msg import SwarmStatus, Capability

class CapabilityManager(Node):
    def __init__(self):
        super().__init__('capability_manager')
        
        # Parameters
        self.declare_parameter('robot_id', 'tb3_0')
        self.robot_id = self.get_parameter('robot_id').value
        
        # Internal State
        self.current_speed = 0.0
        self.target_speed = 0.0
        self.battery_level = 1.0  # Simulating 100%
        self.fault_map = {}       # Stores active faults

        # Subscribers
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.create_subscription(String, 'inject_fault', self.fault_callback, 10)

        # Publisher
        # Publishes to a global topic, but listeners filter by ID
        self.swarm_pub = self.create_publisher(SwarmStatus, '/swarm/status', 10)
        
        # Timer
        self.create_timer(1.0, self.publish_status)

    def cmd_callback(self, msg):
        # What we WANT to do
        self.target_speed = msg.linear.x

    def odom_callback(self, msg):
        # What we are ACTUALLY doing
        self.current_speed = msg.twist.twist.linear.x

    def fault_callback(self, msg):
        # e.g., "mobility_failure" or "reset"
        fault = msg.data
        if fault == "reset":
            self.fault_map.clear()
        else:
            self.fault_map[fault] = True
        self.get_logger().warn(f"Fault State Changed: {self.fault_map}")

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

    def publish_status(self):
        msg = SwarmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.robot_id

        # MOBILITY
        mob = Capability()
        mob.type = "MOBILITY"
        mob.value = self.calculate_mobility()
        mob.is_degraded = mob.value < 0.5
        msg.capabilities.append(mob)

        # BATTERY (Simulated decay)
        self.battery_level = max(0.0, self.battery_level - 0.001)
        bat = Capability()
        bat.type = "BATTERY"
        bat.value = self.battery_level
        bat.is_degraded = self.battery_level < 0.2
        msg.capabilities.append(bat)

        self.swarm_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CapabilityManager()
    rclpy.spin(node)
    rclpy.shutdown()
import rclpy
from rclpy.node import Node
import importlib

from sensor_msgs.msg import BatteryState
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from cares_interfaces.msg import Capability, TaskAllocation

class StandardTelemetryBridge(Node):
    def __init__(self):
        super().__init__('standard_telemetry_bridge')
        self.declare_parameter('robot_id', 'unknown')
        self.robot_id = self.get_parameter('robot_id').value
        
        # Battery Config
        self.declare_parameter('telemetry.has_battery', False)
        self.has_battery = self.get_parameter('telemetry.has_battery').value
        
        if self.has_battery:
            self.declare_parameter('telemetry.battery_topic', 'battery_state')
            self.declare_parameter('telemetry.battery_v_max', 12.6)
            self.declare_parameter('telemetry.battery_v_min', 11.1)
            
            self.v_max = self.get_parameter('telemetry.battery_v_max').value
            self.v_min = self.get_parameter('telemetry.battery_v_min').value
            bat_topic = self.get_parameter('telemetry.battery_topic').value
            
            self.create_subscription(BatteryState, bat_topic, self.battery_cb, 10)
        else:
            self.declare_parameter('telemetry.drain_rate_idle', 0.0005)
            self.declare_parameter('telemetry.drain_rate_active', 0.002)
            self.drain_idle = self.get_parameter('telemetry.drain_rate_idle').value
            self.drain_active = self.get_parameter('telemetry.drain_rate_active').value
            
            self.simulated_battery_level = 1.0
            self.is_active = False
            self.create_subscription(TaskAllocation, '/swarm/allocation', self.allocation_cb, 10)

        # Watchdog Configs
        string_array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        double_array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)

        self.declare_parameter('telemetry.watchdogs.topics', value=None, descriptor=string_array_desc)
        self.declare_parameter('telemetry.watchdogs.types', value=None, descriptor=string_array_desc)
        self.declare_parameter('telemetry.watchdogs.capabilities', value=None, descriptor=string_array_desc)
        self.declare_parameter('telemetry.watchdogs.timeouts', value=None, descriptor=double_array_desc)

        # 3. Retrieve values, and default to empty lists in Python if the YAML doesn't have them
        topics_val = self.get_parameter('telemetry.watchdogs.topics').value
        types_val = self.get_parameter('telemetry.watchdogs.types').value
        caps_val = self.get_parameter('telemetry.watchdogs.capabilities').value
        timeouts_val = self.get_parameter('telemetry.watchdogs.timeouts').value

        topics = topics_val if topics_val is not None else []
        types = types_val if types_val is not None else []
        caps = caps_val if caps_val is not None else []
        timeouts = timeouts_val if timeouts_val is not None else []

        self.watchdog_state = {}

        # Dynamically generate subscriptions
        for i in range(len(topics)):
            topic = topics[i]
            msg_type_str = types[i]
            cap_name = caps[i]
            timeout = timeouts[i]

            # Dynamically import the message type (e.g., "sensor_msgs.msg.LaserScan")
            msg_class = self.get_msg_class(msg_type_str)
            
            if msg_class is None:
                self.get_logger().error(f"Failed to load message type {msg_type_str} for topic {topic}")
                continue

            # Initialize state tracking for this capability
            self.watchdog_state[cap_name] = {
                'last_seen': self.get_clock().now(),
                'timeout': timeout,
                'healthy': True
            }

            # Create the dynamic subscription
            # We use a lambda to pass the specific capability name into the generic callback
            cb = lambda msg, c=cap_name: self.watchdog_cb(msg, c)
            self.create_subscription(msg_class, topic, cb, 10)
            
            self.get_logger().info(f"Tracking {topic} ({msg_type_str}) -> {cap_name} [Timeout: {timeout}s]")

        # Publisher
        self.telemetry_pub = self.create_publisher(Capability, 'telemetry', 10)

        # Health Evaluation Loop (Runs at 2Hz)
        self.create_timer(0.5, self.evaluate_health)
        self.get_logger().info("Dynamic Telemetry Bridge Active.")

    def get_msg_class(self, type_string):
        try:
            parts = type_string.split('.')
            module_name = '.'.join(parts[:-1])
            class_name = parts[-1]
            module = importlib.import_module(module_name)
            return getattr(module, class_name)
        except Exception as e:
            self.get_logger().error(f"Dynamic import error: {e}")
            return None

    def battery_cb(self, msg):
        normalized = (msg.voltage - self.v_min) / (self.v_max - self.v_min)
        normalized = max(0.0, min(1.0, normalized))
        self.publish_cap("BATTERY", normalized)

    def allocation_cb(self, msg):
        if msg.robot_id == self.robot_id:
            if msg.status == "ASSIGNED":
                self.is_active = True
            elif msg.status in ["COMPLETED", "ABORTED", "FAILED"]:
                self.is_active = False

    def watchdog_cb(self, msg, cap_name):
        if cap_name in self.watchdog_state:
            self.watchdog_state[cap_name]['last_seen'] = self.get_clock().now()

    def simulate_battery_drain(self):
        drain = self.drain_active if self.is_active else self.drain_idle
        self.simulated_battery_level = max(0.0, self.simulated_battery_level - drain)
        self.publish_cap("BATTERY", self.simulated_battery_level)

    def evaluate_health(self):
        now = self.get_clock().now()

        if not self.has_battery:
            self.simulate_battery_drain()

        for cap_name, state in self.watchdog_state.items():
            time_since_last_msg = (now - state['last_seen']).nanoseconds / 1e9
            
            # If we haven't heard from the sensor in a while, it's dead
            if time_since_last_msg > state['timeout']:
                if state['healthy']:
                    self.get_logger().warn(f"Telemetry Watchdog: {cap_name} timed out! Degrading capability.")
                    state['healthy'] = False
                self.publish_cap(cap_name, 0.1) # Critically degraded
            else:
                if not state['healthy']:
                    self.get_logger().info(f"Telemetry Watchdog: {cap_name} recovered!")
                    state['healthy'] = True
                self.publish_cap(cap_name, 1.0) # Healthy

    def publish_cap(self, type_name, value):
        msg = Capability()
        msg.type = type_name
        msg.value = float(value)
        self.telemetry_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(StandardTelemetryBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
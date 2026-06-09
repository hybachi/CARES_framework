"""
telemetry_bridge.py
Monitors raw robot telemetry topics using watchdogs to infer capability health.

Author: H.A. Sharif
Year: 2026
"""

import rclpy
import importlib
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from chroma_interfaces.msg import Capability, TaskAllocation

QOS_PRESETS = {
    'reliable': QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
    ),
    'best_effort': QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
    ),
}

class TelemetryBridge(Node):
    def __init__(self):
        super().__init__('telemetry_bridge')
        self.setup_params()
        self.setup_watchdogs()
        self.setup_pubs_timers()
        self.get_logger().info(f"[{self.robot_id}] Telemetry Bridge Active")

    # -------- Setup --------

    def setup_params(self):
        self.declare_parameter('robot_id', 'unknown')
        self.robot_id = self.get_parameter('robot_id').value

        self.declare_parameter('telemetry.has_battery', False)
        self.has_battery = self.get_parameter('telemetry.has_battery').value

        if self.has_battery:
            self.setup_real_battery()
        else:
            self.setup_sim_battery()

    def setup_real_battery(self):
        self.declare_parameter('telemetry.battery_topic', 'battery_state')
        self.declare_parameter('telemetry.battery_v_max', 12.6)
        self.declare_parameter('telemetry.battery_v_min', 11.1)

        self.v_max = self.get_parameter('telemetry.battery_v_max').value
        self.v_min = self.get_parameter('telemetry.battery_v_min').value
        battery_topic = self.get_parameter('telemetry.battery_topic').value
        
        self.create_subscription(BatteryState, battery_topic, self.battery_cb, 10)

    def setup_sim_battery(self):
        self.is_active = False
        self.sim_battery_level = 1.0

        self.declare_parameter('telemetry.drain_rate_idle', 0.0005)
        self.declare_parameter('telemetry.drain_rate_active', 0.002)
        self.drain_idle = self.get_parameter('telemetry.drain_rate_idle').value
        self.drain_active = self.get_parameter('telemetry.drain_rate_active').value

        # Monitor active tasks to calculate battery drain
        self.create_subscription(TaskAllocation, '/swarm/allocation', self.alloc_cb, 10)

    def setup_watchdogs(self):
        self.watchdogs = {}

        string_array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        double_array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)

        self.declare_parameter('telemetry.watchdogs.topics', value=None, descriptor=string_array_desc)
        self.declare_parameter('telemetry.watchdogs.types', value=None, descriptor=string_array_desc)
        self.declare_parameter('telemetry.watchdogs.capabilities', value=None, descriptor=string_array_desc)
        self.declare_parameter('telemetry.watchdogs.timeouts', value=None, descriptor=double_array_desc)
        self.declare_parameter('telemetry.watchdogs.reliability', value=None, descriptor=string_array_desc)

        # Use fallbacks to prevent missing config crashes
        topics = self.get_safe_list('telemetry.watchdogs.topics', [])
        types = self.get_safe_list('telemetry.watchdogs.types', [])
        caps = self.get_safe_list('telemetry.watchdogs.capabilities', [])
        timeouts = self.get_safe_list('telemetry.watchdogs.timeouts', [])
        reliability = self.get_safe_list('telemetry.watchdogs.reliability', ['reliable'] * len(topics))

        for i, topic in enumerate(topics):
            msg_type_str = types[i]
            cap_name = caps[i]
            timeout = timeouts[i]
            qos_key = reliability[i] if i < len(reliability) else 'reliable'
            qos_profile = QOS_PRESETS.get(qos_key, QOS_PRESETS['reliable'])

            msg_class = self.get_msg_class(msg_type_str)
            if msg_class is None:
                self.get_logger().error(f"Failed to load message type '{msg_type_str}' for topic '{topic}'.")
                continue

            self.watchdogs[cap_name] = {
                'last_seen': self.get_clock().now(),
                'timeout': timeout,
                'healthy': True,
            }

            # Bind 'c' to prevent late-binding lambda issues
            callback = lambda msg, c=cap_name: self.watchdog_cb(msg, c)
            self.create_subscription(msg_class, topic, callback, qos_profile)

            self.get_logger().info(
                f"Tracking {topic} ({msg_type_str}) -> {cap_name} "
                f"[Timeout: {timeout}s, QoS: {qos_key}]"
            )

    def setup_pubs_timers(self):
        self.telemetry_pub = self.create_publisher(Capability, 'telemetry', 10)
        self.create_timer(0.5, self.evaluate_health)

    # -------- Callbacks --------

    def battery_cb(self, msg):
        normalized = (msg.voltage - self.v_min) / (self.v_max - self.v_min)
        normalized = max(0.0, min(1.0, normalized))
        self.publish_cap("BATTERY", normalized)

    def alloc_cb(self, msg):
        if msg.robot_id != self.robot_id:
            return
        
        if msg.status == "ASSIGNED":
            self.is_active = True
        elif msg.status in ["COMPLETED", "ABORTED", "FAILED"]:
            self.is_active = False

    def watchdog_cb(self, msg, cap_name):
        if cap_name in self.watchdogs:
            self.watchdogs[cap_name]['last_seen'] = self.get_clock().now()

    # -------- Logic --------

    def evaluate_health(self):
        now = self.get_clock().now()

        if not self.has_battery:
            self.simulate_battery_drain()

        for cap_name, state in self.watchdogs.items():
            time_since_last_msg = (now - state['last_seen']).nanoseconds / 1e9

            if time_since_last_msg > state['timeout'] and state['healthy']:
                self.get_logger().warn(f"Telemetry Watchdog: {cap_name} timed out! Degrading capability.")
                state['healthy'] = False
                self.publish_cap(cap_name, 0.1)

            elif not state['healthy']:
                self.get_logger().info(f"Telemetry Watchdog: {cap_name} recovered!")
                state['healthy'] = True
                self.publish_cap(cap_name, 1.0)
            
    def simulate_battery_drain(self):
        drain = self.drain_active if self.is_active else self.drain_idle
        self.sim_battery_level = max(0.0, self.sim_battery_level - drain)
        self.publish_cap("BATTERY", self.sim_battery_level)

    def publish_cap(self, type_string, value):
        msg = Capability()
        msg.type  = type_string
        msg.value = float(value)
        self.telemetry_pub.publish(msg)

    # -------- Helpers --------

    def get_safe_list(self, param_name, default_val):
        val = self.get_parameter(param_name).value
        return val if val is not None else default_val

    def get_msg_class(self, type_string):
        # Dynamic import allows reusing bridge across topics
        try:
            parts = type_string.split('.')
            module_name = '.'.join(parts[:-1])
            class_name = parts[-1]
            module = importlib.import_module(module_name)
            return getattr(module, class_name)
        except Exception as e:
            self.get_logger().error(f"Dynamic import error for {type_string}: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
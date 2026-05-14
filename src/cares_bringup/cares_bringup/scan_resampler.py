import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

class ScanResampler(Node):
    def __init__(self):
        super().__init__('scan_resampler')
        self.declare_parameter('num_ranges',  360)
        self.declare_parameter('angle_min',  -3.14159)
        self.declare_parameter('angle_max',   3.14159)
        self.num_ranges = self.get_parameter('num_ranges').value
        self.angle_min  = self.get_parameter('angle_min').value
        self.angle_max  = self.get_parameter('angle_max').value

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub = self.create_publisher(LaserScan, 'scan_fixed', pub_qos)
        self.sub = self.create_subscription(LaserScan, 'scan', self.cb, sub_qos)
        self.get_logger().info(
            f'Scan resampler active: → {self.num_ranges} fixed ranges'
        )

    def cb(self, msg):
        angle_increment = (self.angle_max - self.angle_min) / (self.num_ranges - 1)

        out_ranges = []
        out_intensities = []

        for i in range(self.num_ranges):
            angle = self.angle_min + i * angle_increment

            idx = int((angle - msg.angle_min) / msg.angle_increment)
            idx = max(0, min(len(msg.ranges) - 1, idx))
            out_ranges.append(msg.ranges[idx])
            if msg.intensities:
                out_intensities.append(msg.intensities[idx])

        out = LaserScan()
        out.header = msg.header
        out.angle_min = self.angle_min
        out.angle_max = self.angle_max
        out.angle_increment = angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = out_ranges
        out.intensities = out_intensities

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ScanResampler())
    rclpy.shutdown()
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import cares_dashboard.state as state
import random

from cares_interfaces.msg import SwarmStatus, Task, Bid, TaskAllocation
from geometry_msgs.msg import Point
from std_msgs.msg import String

class RosBridge(Node):
    def __init__(self):
        super().__init__('dashboard_bridge')

        # QoS for State
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # SUBSCRIBERS
        self.create_subscription(SwarmStatus, '/swarm/status', self.status_callback, state_qos)
        self.create_subscription(TaskAllocation, '/swarm/allocation', self.allocation_callback, 10)
        self.create_subscription(String, '/swarm/logs', self.swarm_log_callback, 10)

        # PUBLISHERS   
        self.task_pub = self.create_publisher(Task, '/mission/tasks', 10)

        self.get_logger().info("Dashboard Bridge Connected to ROS 2 Network")

    def status_callback(self, msg):
        rid = msg.robot_id
        robot = state.get_robot(rid)
        
        # TODO: add robot types
        if 'tb3' in rid:
            robot['type'] = 'UGV'
        elif 'drone' in rid or 'crazy' in rid:
            robot['type'] = 'UAV'
        
        robot['status'] = 'ONLINE'

        for cap in msg.capabilities:
            robot['caps'][cap.type] = cap.value
            
            if cap.type == 'BATTERY':
                robot['battery'] = cap.value * 100.0 # Convert 0-1 to percent

        robot['history']['mob'].append(robot['caps'].get('MOBILITY', 0.0))
        robot['history']['sen'].append(robot['caps'].get('SENSING', 0.0)) 
        robot['history']['net'].append(robot['caps'].get('NETWORK', 0.0))

        for k in robot['history']: 
            if len(robot['history'][k]) > 50: 
                robot['history'][k].pop(0)

        self.update_metrics()

    def allocation_callback(self, msg):
        if msg.status == "ABORTED":
            for alloc in state.allocations:
                if alloc['id'] == msg.task_id:
                    alloc['status'] = 'ABORTED'
                    alloc['winner'] = msg.robot_id
                    break
            return

        log_msg = f"Task {msg.task_id} {msg.status} to {msg.robot_id}"
        state.log_event(log_msg)
        
        state.allocations.insert(0, {
            'id': msg.task_id,
            'type': 'UNKNOWN', # TODO: get task type
            'status': msg.status,
            'winner': msg.robot_id,
            'cost': 'N/A' # TODO: display cost
        })

        if msg.status == 'ASSIGNED':
             robot = state.get_robot(msg.robot_id)
             robot['status'] = 'WORKING'

    def swarm_log_callback(self, msg):
        state.log_event(msg.data)   

    def update_metrics(self):
        active_bots = [r for r in state.robots.values() if r['status'] != 'OFFLINE']
        state.metrics['active_units'] = len(active_bots)
        
        if active_bots:
            total_mob = sum(r['caps'].get('MOBILITY', 0) for r in active_bots)
            state.metrics['swarm_mean_cap'] = round(total_mob / len(active_bots), 2)

    def dispatch_mission(self, task_type, x, y, prio):
        t_id = f"T-{random.randint(1000, 9999)}"
        
        msg = Task()
        msg.task_id = t_id
        msg.type = task_type
        msg.priority = float(prio)
        msg.location = Point(x=float(x), y=float(y), z=0.0)
        
        # TODO: add mission requirements
        if task_type == 'SEARCH':
            msg.required_capabilities = ['MOBILITY']
            msg.min_capability_score = 0.5
        elif task_type == 'SCAN':
             msg.required_capabilities = ['VISION']
             msg.min_capability_score = 0.7

        self.task_pub.publish(msg)
        state.log_event(f"Mission Dispatched: {t_id}")
        return t_id
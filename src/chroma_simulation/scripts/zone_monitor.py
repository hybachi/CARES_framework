#!/usr/bin/env python3

"""
zone_monitor.py
Monitors robot poses and injects configured faults when they enter hazardous areas.

Author: H.A. Sharif
Year: 2026
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from functools import partial
from ament_index_python.packages import get_package_share_directory

class ZoneMonitor(Node):
    def __init__(self):
        super().__init__('zone_monitor')
        self.setup_state()
        self.setup_pubs_subs()

    # -------- Setup --------

    def setup_state(self):
        self.robot_states = {}
        self.fault_pubs = {}
        
        sim_dir = get_package_share_directory("chroma_simulation")
        config_path = os.path.join(sim_dir, "config", "swarm_config.yaml")
        
        with open(config_path, 'r') as f:
            self.swarm_data = yaml.safe_load(f).get('swarm', {})

    def setup_pubs_subs(self):
        for robot_id, params in self.swarm_data.items():
            if 'hazard_zone' not in params:
                continue

            self.robot_states[robot_id] = {
                'in_hazard': False,
                'rule': params['hazard_zone']
            }

            pub_topic = f'/{robot_id}/inject_fault'
            self.fault_pubs[robot_id] = self.create_publisher(String, pub_topic, 10)

            sub_topic = f'/{robot_id}/odom'
            cb = partial(self.odom_cb, robot_id=robot_id)
            self.create_subscription(Odometry, sub_topic, cb, 10)
            
            self.get_logger().info(f"Tracking {robot_id} for Hazard Zone")

    # -------- Callbacks --------

    def odom_cb(self, msg, robot_id):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        state = self.robot_states[robot_id]
        rule = state['rule']
        
        x_min = rule.get('x_min', float('-inf'))
        x_max = rule.get('x_max', float('inf'))
        y_min = rule.get('y_min', float('-inf'))
        y_max = rule.get('y_max', float('inf'))
        
        in_zone = (x_min <= x <= x_max) and (y_min <= y <= y_max)

        if in_zone and not state['in_hazard']:
            state['in_hazard'] = True
            self.get_logger().warn(f"[{robot_id}] entered hazard zone!")
            self.send_fault(robot_id, rule['fault'])
            
        elif not in_zone and state['in_hazard']:
            state['in_hazard'] = False
            self.get_logger().info(f"[{robot_id}] returned to Safe Zone.")
            self.send_fault(robot_id, "reset")

    # -------- Helper --------

    def send_fault(self, robot_id, fault_type):
        msg = String()
        msg.data = fault_type
        self.fault_pubs[robot_id].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZoneMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
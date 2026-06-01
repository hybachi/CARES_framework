#!/usr/bin/env python3
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
        
        chroma_sim_dir = get_package_share_directory("chroma_simulation")
        swarm_config_path = os.path.join(chroma_sim_dir, "config", "swarm_config.yaml")
        
        with open(swarm_config_path, 'r') as f:
            swarm_data = yaml.safe_load(f).get('swarm', {})

        self.robot_states = {}
        self.publishers_dict = {}

        for robot_id, params in swarm_data.items():
            if 'hazard_zone' in params:
                rule = params['hazard_zone']
                
                self.robot_states[robot_id] = {
                    'in_hazard': False,
                    'rule': rule
                }

                pub_topic = f'/{robot_id}/inject_fault'
                self.publishers_dict[robot_id] = self.create_publisher(String, pub_topic, 10)

                sub_topic = f'/{robot_id}/odom'
                self.create_subscription(
                    Odometry,
                    sub_topic,
                    partial(self.odom_callback, robot_id=robot_id), 
                    10
                )
                
                self.get_logger().info(f"Tracking {robot_id} for Hazard Zone")

    def odom_callback(self, msg, robot_id):
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
            self.get_logger().warn(f"[{robot_id}] entered hazard zone! Injecting: {rule['fault']}")
            
            fault_msg = String()
            fault_msg.data = rule['fault']
            self.publishers_dict[robot_id].publish(fault_msg)
            
        elif not in_zone and state['in_hazard']:
            state['in_hazard'] = False
            self.get_logger().info(f"[{robot_id}] returned to Safe Zone. Clearing faults.")
            
            clear_msg = String()
            clear_msg.data = "reset"
            self.publishers_dict[robot_id].publish(clear_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ZoneMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
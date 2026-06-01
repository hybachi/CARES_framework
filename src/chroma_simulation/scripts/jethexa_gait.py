#!/usr/bin/env python3

"""
jethexa_gait.py
Translates twist commands into continuous joint positions for a hexapod robot.

Author: H.A. Sharif
Year: 2026
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class JethexaGaitController(Node):
    def __init__(self):
        super().__init__('jethexa_gait_controller')
        self.setup_state()
        self.setup_pubs_subs()
        self.setup_timers()
        self.get_logger().info("Hexapod Gait Controller Ready")

    # -------- Setup --------

    def setup_state(self):
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.is_moving = False
        self.phase = 0.0

        self.legs = ['LF', 'LM', 'LR', 'RF', 'RM', 'RR']
        self.tripod_1 = ['LF', 'RM', 'LR']
        self.tripod_2 = ['RF', 'LM', 'RR']

        self.rest_coxa = 0.0
        self.rest_femur = 0.2   
        self.rest_tibia = -0.4   

        self.linear_scale = 0.35    
        self.angular_scale = 0.35   
        self.max_swing = 0.35  
        self.max_lift = 0.25  
        self.tibia_ratio = 0.4

        self.cycle_speed = (2.0 * math.pi) / 0.6 

        # Scales front/back legs to maintain straight trajectory
        self.geometry_multiplier = {
            'LF': 1.414, 'LM': 1.0, 'LR': 1.414,
            'RF': 1.414, 'RM': 1.0, 'RR': 1.414
        }
        self.joint_pubs = {}

    def setup_pubs_subs(self):
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)

        for leg in self.legs:
            for joint in ['coxa', 'femur', 'tibia']:
                j_name = f"{joint}_joint_{leg}"
                self.joint_pubs[j_name] = self.create_publisher(
                    Float64, f"joint/{j_name}/cmd_pos", 10
                )

    def setup_timers(self):
        self.create_timer(0.02, self.update_gait)

    # -------- Callbacks --------

    def cmd_vel_cb(self, msg: Twist):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z
        self.is_moving = abs(self.linear_x) > 0.01 or abs(self.angular_z) > 0.01

    # -------- Logic --------

    def publish_joint(self, joint_name: str, position: float):
        msg = Float64()
        msg.data = float(position)
        self.joint_pubs[joint_name].publish(msg)

    def update_gait(self):
        dt = 0.02

        if self.is_moving:
            self.phase = (self.phase + self.cycle_speed * dt) % (2 * math.pi)
        else:
            # Settle phase gracefully to zero to prevent slipping
            if self.phase > 0.1:
                if self.phase > math.pi:
                    self.phase = (self.phase + self.cycle_speed * dt) % (2 * math.pi)
                else:
                    self.phase = max(0.0, self.phase - self.cycle_speed * dt)

        lift_1 = max(0.0, math.sin(self.phase))
        swing_1 = -math.cos(self.phase)

        lift_2 = max(0.0, -math.sin(self.phase))
        swing_2 = math.cos(self.phase)

        speed_mag = math.hypot(self.linear_x, self.angular_z)
        dynamic_lift = min(self.max_lift, 0.05 + (speed_mag * 0.4))

        for leg in self.legs:
            is_tripod_1 = leg in self.tripod_1
            lift_factor = lift_1 if is_tripod_1 else lift_2
            swing_factor = swing_1 if is_tripod_1 else swing_2

            if not self.is_moving and self.phase == 0.0:
                lift_factor = 0.0
                swing_factor = 0.0

            if leg in ['LF', 'LM', 'LR']:
                target_offset = (-self.linear_x * self.linear_scale) + (self.angular_z * self.angular_scale)
            else:
                target_offset = (self.linear_x * self.linear_scale) + (self.angular_z * self.angular_scale)

            if not self.is_moving:
                target_offset *= 0.1

            target_offset = max(min(target_offset, self.max_swing), -self.max_swing)
            leg_swing = swing_factor * target_offset * self.geometry_multiplier[leg]

            self.publish_joint(f"coxa_joint_{leg}", self.rest_coxa + leg_swing)
            self.publish_joint(f"femur_joint_{leg}", self.rest_femur + (lift_factor * dynamic_lift))
            self.publish_joint(f"tibia_joint_{leg}", self.rest_tibia - (lift_factor * dynamic_lift * self.tibia_ratio))


def main(args=None):
    rclpy.init(args=args)
    controller = JethexaGaitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
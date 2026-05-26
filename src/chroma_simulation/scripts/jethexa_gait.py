#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class JethexaGaitController(Node):
    def __init__(self):
        super().__init__('jethexa_gait_controller')

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.legs = ['LF', 'LM', 'LR', 'RF', 'RM', 'RR']
        self.pubs = {}
        for leg in self.legs:
            for joint in ['coxa', 'femur', 'tibia']:
                joint_name = f"{joint}_joint_{leg}"
                self.pubs[joint_name] = self.create_publisher(Float64, f"joint/{joint_name}/cmd_pos", 10)

        self.timer = self.create_timer(0.02, self.control_loop) 

        # --- STABLE POSTURE SETTINGS ---
        self.rest_coxa   =  0.0
        self.rest_femur  =  0.2   
        self.rest_tibia  = -0.4   

        # --- ARC / SWAY REDUCTION ---
        self.linear_scale  = 0.35    
        self.angular_scale = 0.35   
        
        # Hard limits
        self.max_swing_limit = 0.35  
        self.max_lift_limit  = 0.25  
        self.tibia_ratio     = 0.4

        # Fixed 0.6 seconds per cycle
        self.period = 0.6 
        self.cycle_speed = (2.0 * math.pi) / self.period 

        self.tripod_1 = ['LF', 'RM', 'LR']
        self.tripod_2 = ['RF', 'LM', 'RR']

        # GEOMETRY FIX: Front/Back legs must swing 1.414x wider to match Middle legs' forward distance
        self.geometry_multiplier = {
            'LF': 1.414, 'LM': 1.0, 'LR': 1.414,
            'RF': 1.414, 'RM': 1.0, 'RR': 1.414
        }

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.is_moving = False
        self.phase = 0.0

        self.get_logger().info("Hexapod Gait Controller Ready! Geometric drift compensation engaged.")

    def cmd_vel_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

        if abs(self.linear_x) > 0.01 or abs(self.angular_z) > 0.01:
            self.is_moving = True
        else:
            self.is_moving = False

    def publish_joint(self, joint_name, position):
        msg = Float64()
        msg.data = float(position)
        self.pubs[joint_name].publish(msg)

    def control_loop(self):
        dt = 0.02

        if self.is_moving:
            self.phase += self.cycle_speed * dt
            if self.phase > 2 * math.pi:
                self.phase -= 2 * math.pi
        else:
            # Smooth stop to prevent slipping
            if self.phase > 0.1:
                if self.phase > math.pi:
                    self.phase += self.cycle_speed * dt
                    if self.phase > 2 * math.pi: self.phase = 0.0
                else:
                    self.phase -= self.cycle_speed * dt
                    if self.phase < 0: self.phase = 0.0

        lift_1 = max(0.0, math.sin(self.phase))
        swing_1 = -math.cos(self.phase)

        lift_2 = max(0.0, -math.sin(self.phase))
        swing_2 = math.cos(self.phase)

        speed_mag = math.hypot(self.linear_x, self.angular_z)
        dynamic_lift = min(self.max_lift_limit, 0.05 + (speed_mag * 0.4))

        for leg in self.legs:
            is_tripod_1 = leg in self.tripod_1
            
            lift_factor = lift_1 if is_tripod_1 else lift_2
            swing_factor = swing_1 if is_tripod_1 else swing_2

            if not self.is_moving and self.phase == 0.0:
                lift_factor = 0.0
                swing_factor = 0.0

            # ----------------------------------------------------
            # DIRECT KINEMATIC MAPPING 
            # ----------------------------------------------------
            if leg in ['LF', 'LM', 'LR']:
                target_offset = (-self.linear_x * self.linear_scale) + (self.angular_z * self.angular_scale)
            else:
                target_offset = (self.linear_x * self.linear_scale) + (self.angular_z * self.angular_scale)

            if not self.is_moving:
                target_offset *= 0.1

            # Clamp baseline offset
            target_offset = max(min(target_offset, self.max_swing_limit), -self.max_swing_limit)

            # APPLY GEOMETRY FIX: Scale the front/back legs up by 1.414
            leg_swing_offset = swing_factor * target_offset * self.geometry_multiplier[leg]

            # Calculate actual joint positions
            coxa_angle = self.rest_coxa + leg_swing_offset
            femur_angle = self.rest_femur + (lift_factor * dynamic_lift)
            tibia_angle = self.rest_tibia - (lift_factor * dynamic_lift * self.tibia_ratio)

            self.publish_joint(f"coxa_joint_{leg}", coxa_angle)
            self.publish_joint(f"femur_joint_{leg}", femur_angle)
            self.publish_joint(f"tibia_joint_{leg}", tibia_angle)

def main(args=None):
    rclpy.init(args=args)
    controller = JethexaGaitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
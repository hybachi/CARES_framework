#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from chroma_interfaces.msg import Task, TaskAllocation
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import math
import time

class MissionExecutor(Node):
    def __init__(self):
        super().__init__('mission_executor')
        self.declare_parameter('robot_id', 'unknown')
        self.robot_id = self.get_parameter('robot_id').value

        self.create_subscription(Task, '/mission/tasks', self.global_task_cb, 10)
        self.create_subscription(TaskAllocation, '/swarm/allocation', self.global_alloc_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        self.alloc_pub = self.create_publisher(TaskAllocation, '/swarm/allocation', 10)
        self.bridge_pub = self.create_publisher(Task, 'execute_waypoint', 10)
        self.create_subscription(TaskAllocation, 'execution_status', self.bridge_status_cb, 10)

        self.global_tasks = {}
        self.current_mission = None
        self.mission_type = ""
        self.current_pose = Point()
        
        self.waypoints = []
        self.current_wp_idx = 0
        self.wait_timer = None

        self.get_logger().info(f"[{self.robot_id}] Mission Executor ready.")

    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose.position

    def global_task_cb(self, msg: Task):
        self.global_tasks[msg.task_id] = msg

    def global_alloc_cb(self, msg: TaskAllocation):
        if msg.robot_id == self.robot_id and msg.status == "ASSIGNED" and self.current_mission is None:
            task_details = self.global_tasks.get(msg.task_id)
            if not task_details: return

            self.current_mission = msg.task_id
            self.mission_type = task_details.type
            self.get_logger().info(f"--- STARTING MISSION: {self.current_mission} ({self.mission_type}) ---")
            
            self.generate_waypoints(task_details)
            self.send_next_waypoint()
            
        elif msg.task_id == self.current_mission and (msg.status == "ABORTED" or msg.status == "CANCELLED"):
            self.get_logger().warn(f"Mission halted ({msg.status}). Dropping execution.")
            self.current_mission = None
            if self.wait_timer:
                self.wait_timer.cancel()

    def generate_waypoints(self, task: Task):
        self.waypoints = []
        self.current_wp_idx = 0

        p1 = task.location
        p2 = task.target_area[0] if len(task.target_area) > 0 else p1

        if task.type == "DELIVERY":
            yaw_ab = math.atan2(p2.y - p1.y, p2.x - p1.x)
            p1.z = yaw_ab
            p2.z = yaw_ab 
            
            self.waypoints.append(p1)
            self.waypoints.append(p2)

        elif task.type == "SEARCH":
            min_x, max_x = min(p1.x, p2.x), max(p1.x, p2.x)
            min_y, max_y = min(p1.y, p2.y), max(p1.y, p2.y)
            
            corners = [
                (min_x, min_y), (max_x, min_y),
                (max_x, max_y), (min_x, max_y)
            ]
            cx, cy = self.current_pose.x, self.current_pose.y
            start_x, start_y = min(corners, key=lambda c: math.hypot(c[0]-cx, c[1]-cy))
            
            width = max_x - min_x
            height = max_y - min_y
            step_size = 0.5 
            
            raw_points = []
            
            if width >= height: 
                y_dir = 1 if start_y == min_y else -1
                curr_y = start_y
                curr_x = start_x
                
                while min_y <= curr_y <= max_y:
                    raw_points.append((curr_x, curr_y))
                    curr_x = max_x if curr_x == min_x else min_x
                    curr_y += (step_size * y_dir)
                    
            else: 
                x_dir = 1 if start_x == min_x else -1
                curr_x = start_x
                curr_y = start_y
                
                while min_x <= curr_x <= max_x:
                    raw_points.append((curr_x, curr_y))
                    curr_y = max_y if curr_y == min_y else min_y
                    curr_x += (step_size * x_dir)

            for i in range(len(raw_points)):
                x1, y1 = raw_points[i]
                
                if i < len(raw_points) - 1:
                    x2, y2 = raw_points[i+1]
                    yaw = math.atan2(y2 - y1, x2 - x1)
                else:
                    yaw = self.waypoints[-1].z if len(self.waypoints) > 0 else 0.0
                
                pt = Point()
                pt.x = float(x1)
                pt.y = float(y1)
                pt.z = float(yaw) 
                self.waypoints.append(pt)

    def send_next_waypoint(self):
        if self.current_mission is None: return

        if self.current_wp_idx < len(self.waypoints):
            wp = self.waypoints[self.current_wp_idx]
            wp_task = Task()
            wp_task.task_id = f"{self.current_mission}_WP_{self.current_wp_idx}"
            wp_task.type = "GOTO"
            wp_task.location = wp
            
            self.get_logger().info(f"Sending Waypoint {self.current_wp_idx + 1}/{len(self.waypoints)}")
            self.bridge_pub.publish(wp_task)
        else:
            self.get_logger().info(f"--- MISSION {self.current_mission} COMPLETED ---")
            complete_msg = TaskAllocation()
            complete_msg.task_id = self.current_mission
            complete_msg.robot_id = self.robot_id
            complete_msg.status = "COMPLETED"
            self.alloc_pub.publish(complete_msg)
            self.current_mission = None

    def bridge_status_cb(self, msg: TaskAllocation):
        if self.current_mission is None: return

        if msg.status == "COMPLETED" or msg.status == "FAILED":
            if msg.status == "FAILED":
                if self.mission_type == "DELIVERY":
                    self.get_logger().error("Failed to reach delivery point. Aborting mission.")
                    fail_msg = TaskAllocation()
                    fail_msg.task_id = self.current_mission
                    fail_msg.robot_id = self.robot_id
                    fail_msg.status = "FAILED"
                    self.alloc_pub.publish(fail_msg)
                    self.current_mission = None
                    return
                elif self.mission_type == "SEARCH":
                    self.get_logger().warn("Waypoint blocked by obstacle. Skipping to next.")
            
            if self.mission_type == "DELIVERY" and msg.status == "COMPLETED":
                action = "Pickup" if self.current_wp_idx == 0 else "Drop-off"
                self.get_logger().info(f"{action} reached. Waiting 5 seconds...")
                self.wait_timer = self.create_timer(5.0, self.on_wait_finished)
            else:
                self.current_wp_idx += 1
                self.send_next_waypoint()

    def on_wait_finished(self):
        if self.wait_timer:
            self.wait_timer.cancel()
            self.wait_timer = None
        self.get_logger().info("Action complete. Resuming mission.")
        self.current_wp_idx += 1
        self.send_next_waypoint()

def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
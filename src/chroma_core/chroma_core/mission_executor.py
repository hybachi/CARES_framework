"""
mission_executor.py
Breaks high-level task types into executable sequences.

Author: H.A. Sharif
Year: 2026
"""

import rclpy
import math
import time
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from chroma_interfaces.msg import Task, TaskAllocation
from chroma_interfaces.action import ExecuteMission

class MissionExecutor(Node):

    def __init__(self):
        super().__init__('mission_executor')

        self.load_params()
        self.setup_state()
        self.setup_pubs_subs()

        self.get_logger().info(f"[{self.robot_id}] Mission Executor Action Server ready.")

    # -------- Setup --------

    def load_params(self):
        self.declare_parameter('robot_id', 'unknown')
        self.robot_id = self.get_parameter('robot_id').value

    def setup_state(self):
        self.current_pose = Point()
        self.current_wp_status = None
        # Allows action and subscription callbacks to run concurrently
        self.cb_group = ReentrantCallbackGroup()

    def setup_pubs_subs(self):
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10, callback_group=self.cb_group)
        self.create_subscription(TaskAllocation, 'execution_status', self.bridge_status_cb, 10, callback_group=self.cb_group)

        self.execute_pub = self.create_publisher(Task, 'execute_waypoint', 10)
        self.cancel_pub = self.create_publisher(Task, 'cancel_waypoint', 10)

        self.action_server = ActionServer(
            self,
            ExecuteMission,
            'execute_mission',
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group
        )

    # -------- Callbacks --------

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose.position

    def bridge_status_cb(self, msg):
        self.current_wp_status = msg.status

    def cancel_cb(self, goal_handle):
        self.get_logger().warn("Preemption requested by Allocator. Halting Execution.")
        self.publish_halt()
        return CancelResponse.ACCEPT

    # -------- Execution Logic --------

    def execute_cb(self, goal_handle):
        task = goal_handle.request.task
        self.get_logger().info(f"Executing Mission: {task.task_id} ({task.type})")

        waypoints = self.generate_waypoints(task)
        total_wps = len(waypoints)

        for i, wp in enumerate(waypoints):
            if goal_handle.is_cancel_requested:
                return self.abort_execution(goal_handle)

            self.current_wp_status = "IN_PROGRESS"
            self.publish_waypoint(task.task_id, i, wp)
            self.publish_feedback(goal_handle, i, total_wps)

            while self.current_wp_status == "IN_PROGRESS":
                if goal_handle.is_cancel_requested:
                    self.publish_halt()
                    return self.abort_execution(goal_handle)
                time.sleep(0.1)

            if self.current_wp_status == "FAILED":
                if task.type == "DELIVERY":
                    self.get_logger().error("Delivery point unreachable. Aborting mission.")
                    goal_handle.abort()
                    return ExecuteMission.Result(success=False)
                elif task.type == "SEARCH":
                    self.get_logger().warn("Waypoint blocked. Skipping to next.")

            if task.type == "DELIVERY" and self.current_wp_status == "COMPLETED":
                action = "Pickup" if i == 0 else "Drop-off"
                self.get_logger().info(f"{action} reached. Waiting 5 seconds...")
                if not self.wait_with_preemption(goal_handle, duration=5.0):
                    return self.abort_execution(goal_handle)

        self.get_logger().info(f"Mission {task.task_id} Completed.")
        goal_handle.succeed()
        return ExecuteMission.Result(success=True)

    # -------- Waypoint Generation --------

    def generate_waypoints(self, task):
        waypoints = []
        p1 = task.location
        p2 = task.target_area[0] if len(task.target_area) > 0 else p1

        if task.type == "DELIVERY":
            yaw_ab = math.atan2(p2.y - p1.y, p2.x - p1.x)
            p1.z = yaw_ab
            p2.z = yaw_ab
            waypoints.append(p1)
            waypoints.append(p2)

        elif task.type == "SEARCH":
            waypoints = self.build_search_pattern(p1, p2)

        return waypoints

    def build_search_pattern(self, p1, p2):
        min_x, max_x = min(p1.x, p2.x), max(p1.x, p2.x)
        min_y, max_y = min(p1.y, p2.y), max(p1.y, p2.y)

        corners = [
            (min_x, min_y), (max_x, min_y),
            (max_x, max_y), (min_x, max_y)
        ]

        cx, cy = self.current_pose.x, self.current_pose.y
        start_x, start_y = min(corners, key=lambda c: math.hypot(c[0] - cx, c[1] - cy))

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
                curr_y += step_size * y_dir
        else:
            x_dir = 1 if start_x == min_x else -1
            curr_x = start_x
            curr_y = start_y

            while min_x <= curr_x <= max_x:
                raw_points.append((curr_x, curr_y))
                curr_y = max_y if curr_y == min_y else min_y
                curr_x += step_size * x_dir

        return self.apply_orientations(raw_points)

    def apply_orientations(self, raw_points):
        waypoints = []

        for i in range(len(raw_points)):
            x1, y1 = raw_points[i]

            if i < len(raw_points) - 1:
                x2, y2 = raw_points[i + 1]
                yaw = math.atan2(y2 - y1, x2 - x1)
            else:
                # Inherit last waypoint's heading at the final point
                yaw = waypoints[-1].z if len(waypoints) > 0 else 0.0

            pt = Point()
            pt.x = float(x1)
            pt.y = float(y1)
            pt.z = float(yaw)
            waypoints.append(pt)

        return waypoints

    # -------- Helpers --------

    def publish_waypoint(self, task_id, index, wp):
        wp_task = Task()
        wp_task.task_id = f"{task_id}_WP_{index}"
        wp_task.type = "GOTO"
        wp_task.location = wp
        self.execute_pub.publish(wp_task)

    def publish_feedback(self, goal_handle, current, total):
        feedback = ExecuteMission.Feedback()
        feedback.progress = float(current) / total
        feedback.status_message = f"Waypoint {current + 1}/{total}"
        goal_handle.publish_feedback(feedback)

    def publish_halt(self):
        halt = Task()
        halt.task_id = "HALT"
        self.cancel_pub.publish(halt)

    def abort_execution(self, goal_handle):
        goal_handle.canceled()
        self.get_logger().info("Execution safely aborted.")
        return ExecuteMission.Result(success=False)

    def wait_with_preemption(self, goal_handle, duration):
        # Checks cancellation every tick during physical interaction delays
        ticks = int(duration / 0.1)
        for _ in range(ticks):
            if goal_handle.is_cancel_requested:
                return False
            time.sleep(0.1)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
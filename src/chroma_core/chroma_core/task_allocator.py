"""
task_allocator.py
Handles distributed bidding and task assignment logic for the robot.

Author: H.A. Sharif
Year: 2026
"""

import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from chroma_interfaces.msg import Task, Bid, TaskAllocation, SwarmStatus

class TaskAllocator(Node):
    def __init__(self):
        super().__init__('task_allocator')
        self.setup_params()
        self.setup_state()
        self.setup_pubs_subs()
        self.setup_timers()
        self.get_logger().info(f"[{self.robot_id}] Task Allocator ready")

    # -------- Setup --------

    def setup_params(self):
        self.declare_parameter('robot_id', 'tb3_0')
        self.robot_id = self.get_parameter('robot_id').value

    def setup_state(self):
        self.my_caps = {}
        self.swarm_knowledge = {}
        self.current_pose = Point() 
        self.bids_received = {} 
        self.assigned_tasks = {}
        self.active_task = None
        self.tasks_bid_on = set()
        self.pending_auctions = set()
        self.bid_timers = {}
        self.auction_timers = {}

    def setup_pubs_subs(self):
        self.create_subscription(SwarmStatus, '/swarm/status', self.cap_cb, 10)
        self.create_subscription(Task, '/mission/tasks', self.task_cb, 10)
        self.create_subscription(Bid, '/swarm/bids', self.bid_cb, 10)
        self.create_subscription(TaskAllocation, '/swarm/allocation', self.alloc_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        self.bid_pub = self.create_publisher(Bid, '/swarm/bids', 10)
        self.log_pub = self.create_publisher(String, '/swarm/logs', 10)
        self.task_pub = self.create_publisher(Task, '/mission/tasks', 10)
        self.alloc_pub = self.create_publisher(TaskAllocation, '/swarm/allocation', 10)

    def setup_timers(self):
        self.create_timer(2.0, self.health_check)

    # -------- Callbacks --------

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose.position

    def cap_cb(self, msg):
        if msg.robot_id not in self.swarm_knowledge:
            self.swarm_knowledge[msg.robot_id] = {}

        for cap in msg.capabilities:
            self.swarm_knowledge[msg.robot_id][cap.type] = cap.value

        if msg.robot_id == self.robot_id:
            self.my_caps = self.swarm_knowledge[self.robot_id]

    def alloc_cb(self, msg):
        if msg.status == "CANCELLED":
            self.get_logger().warn(f"Task {msg.task_id} cancelled")
            self.clear_task_state(msg.task_id)
            if self.active_task and self.active_task.task_id == msg.task_id:
                self.active_task = None
            return

        if msg.status == "ABORTED":
            self.get_logger().warn(f"Task {msg.task_id} aborted")
            self.clear_task_state(msg.task_id)
            return

        self.assigned_tasks[msg.task_id] = msg.robot_id
        self.pending_auctions.discard(msg.task_id)

        if msg.robot_id == self.robot_id and msg.status == "COMPLETED":
            self.active_task = None
            self.publish_log(f"Task {msg.task_id} completed")

    def bid_cb(self, msg):
        if msg.task_id not in self.bids_received:
            self.bids_received[msg.task_id] = {}
        self.bids_received[msg.task_id][msg.robot_id] = msg.score

    def task_cb(self, task):
        if task.task_id in self.assigned_tasks or task.task_id in self.tasks_bid_on or self.active_task is not None:
            return

        if not self.check_eligibility(task):
            return

        self.tasks_bid_on.add(task.task_id)
        
        # Random delay prevents network flooding from simultaneous bids
        delay = random.uniform(0.1, 0.5)

        def on_bid_timer():
            timer = self.bid_timers.pop(task.task_id, None)
            if timer: 
                timer.cancel()
            self.submit_bid(task)

        self.bid_timers[task.task_id] = self.create_timer(delay, on_bid_timer)

    # -------- Logic --------

    def clear_task_state(self, task_id):
        self.assigned_tasks.pop(task_id, None)
        self.tasks_bid_on.discard(task_id)
        self.pending_auctions.discard(task_id)
        self.bids_received.pop(task_id, None)

    def health_check(self):
        if self.active_task is None:
            return
        
        task = self.active_task
        min_req = task.min_capability_score if task.min_capability_score > 0.0 else 0.4

        for req in task.required_capabilities:
            current_score = self.my_caps.get(req, 0.0)
            
            # Buffer prevents task fluctuations near threshold
            threshold = min_req * 0.8

            if current_score < threshold:
                self.publish_log(f"Aborting {task.task_id}: {req} dropped")
                self.abort_task(task)
                return
    
    def publish_log(self, message):
        self.get_logger().info(message)
        msg = String()
        msg.data = f"[{self.robot_id}] {message}"
        self.log_pub.publish(msg)

    def check_eligibility(self, task):
        min_req = task.min_capability_score if task.min_capability_score > 0.0 else 0.4

        for req in task.required_capabilities:
            my_score = self.my_caps.get(req, 0.0)
            if my_score < min_req:
                self.publish_log(f"Ineligible for {task.task_id}: {req} too low")
                return False
        return True

    def calculate_score(self, task):
        cap_sum = sum(self.my_caps.get(req, 0.0) for req in task.required_capabilities)
        avg_cap = cap_sum / max(1, len(task.required_capabilities))

        # Manhattan distance is cheaper than hypotenuse for heuristic
        dist = abs(task.location.x - self.current_pose.x) + abs(task.location.y - self.current_pose.y)
        dist_factor = 1.0 / (1.0 + dist)

        priority_norm = (task.priority if task.priority > 0 else 1.0) / 10.0

        return avg_cap * priority_norm * dist_factor

    def submit_bid(self, task):
        if task.task_id in self.assigned_tasks or self.active_task is not None:
            return

        my_score = self.calculate_score(task)

        bid_msg = Bid()
        bid_msg.task_id = task.task_id
        bid_msg.robot_id = self.robot_id
        bid_msg.score = my_score
        self.bid_pub.publish(bid_msg)

        if task.task_id not in self.bids_received:
            self.bids_received[task.task_id] = {}
        self.bids_received[task.task_id][self.robot_id] = my_score

        self.publish_log(f"Bid {my_score:.2f} on Task {task.task_id}")
        self.pending_auctions.add(task.task_id)

        def on_auction_timer():
            timer = self.auction_timers.pop(task.task_id, None)
            if timer: 
                timer.cancel()
            self.resolve_auction(task)

        # Fixed auction window gives slow networks time to bid
        self.auction_timers[task.task_id] = self.create_timer(2.0, on_auction_timer)

    def resolve_auction(self, task):
        if task.task_id not in self.pending_auctions or task.task_id in self.assigned_tasks:
            self.pending_auctions.discard(task.task_id)
            return

        all_bids = self.bids_received.get(task.task_id, {})
        highest_score = -1.0
        winner_id = ""

        # Alphabetical sorting for tie-breaking
        for rid, score in all_bids.items():
            if score > highest_score:
                highest_score = score
                winner_id = rid
            elif score == highest_score and rid < winner_id:
                winner_id = rid

        self.pending_auctions.discard(task.task_id)

        if winner_id == self.robot_id:
            self.publish_log(f"I won Task {task.task_id}!")
            self.claim_task(task)
        elif winner_id:
            self.publish_log(f"I lost Task {task.task_id} to {winner_id}")

    def claim_task(self, task):
        self.active_task = task
        self.assigned_tasks[task.task_id] = self.robot_id

        msg = TaskAllocation()
        msg.task_id = task.task_id
        msg.robot_id = self.robot_id
        msg.status = "ASSIGNED"
        self.alloc_pub.publish(msg)

    def abort_task(self, task):
        msg = TaskAllocation()
        msg.task_id = task.task_id
        msg.robot_id = self.robot_id
        msg.status = "ABORTED"
        self.alloc_pub.publish(msg)

        self.active_task = None
        self.clear_task_state(task.task_id)

        self.publish_log(f"Re-auctioning {task.task_id}")
        self.task_pub.publish(task)


def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
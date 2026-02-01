import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from cares_interfaces.msg import Task, Bid, TaskAllocation, SwarmStatus
from geometry_msgs.msg import Point
import math
import random

class TaskAllocator(Node):
    def __init__(self):
        super().__init__('task_allocator')
        
        # Parameters
        self.declare_parameter('robot_id', 'tb3_0')
        self.robot_id = self.get_parameter('robot_id').value
        
        # State
        self.my_capabilities = {} 
        self.current_pose = Point() 
        self.bids_received = {} 
        self.assigned_tasks = {} 
        self.my_active_task = None
        
        # Subscribers
        self.create_subscription(SwarmStatus, '/swarm/status', self.cap_callback, 10)
        self.create_subscription(Task, '/mission/tasks', self.task_callback, 10)
        self.create_subscription(Bid, '/swarm/bids', self.bid_listen_callback, 10)
        self.create_subscription(TaskAllocation, '/swarm/allocation', self.allocation_callback, 10)
        
        # Publishers
        self.bid_pub = self.create_publisher(Bid, '/swarm/bids', 10)
        self.alloc_pub = self.create_publisher(TaskAllocation, '/swarm/allocation', 10)

    def cap_callback(self, msg):
        # Update self
        if msg.robot_id == self.robot_id:
            for cap in msg.capabilities:
                self.my_capabilities[cap.type] = cap.value

    def allocation_callback(self, msg):
        # Update the allocated tasks
        self.assigned_tasks[msg.task_id] = msg.robot_id
        
        if msg.robot_id == self.robot_id and msg.status == "COMPLETED":
            self.my_active_task = None
            self.get_logger().info(f"I finished Task {msg.task_id}")

    def bid_listen_callback(self, msg):
        # Store bids from other robots
        if msg.task_id not in self.bids_received:
            self.bids_received[msg.task_id] = {}
        self.bids_received[msg.task_id][msg.robot_id] = msg.score

    def task_callback(self, task):
        self.get_logger().info(f"Task received: {task.task_id}")

        # Task already assigned?
        if task.task_id in self.assigned_tasks:
            return

        # Task already active?
        if self.my_active_task is not None:
            return

        # Eligible for task?
        if not self.check_eligibility(task):
            return

        # Start bidding
        delay = random.uniform(0.1, 0.5)
        self.get_logger().info(f"Preparing bid for {task.task_id} in {delay:.2f}s")

        def bid_timer(task):
            self._bid_timer.cancel()
            self.submit_bid(task)

        self._bid_timer = self.create_timer(delay, lambda: bid_timer(task))

    def check_eligibility(self, task):
        for req in task.required_capabilities:
            my_score = self.my_capabilities.get(req, 0.0)
            if my_score < task.min_capability_score:
                return False # Not qualified
        return True

    def calculate_score(self, task):
        # Score = (Capabilities * Priority) / Distance
        cap_sum = 0.0
        for req in task.required_capabilities:
            cap_sum += self.my_capabilities.get(req, 0.0)
        avg_cap = cap_sum / max(1, len(task.required_capabilities))
        
        # Manhattan Distance
        dist = abs(task.location.x - self.current_pose.x) + abs(task.location.y - self.current_pose.y)
        dist_factor = 1.0 / (1.0 + dist) # closer = higher score
        
        return avg_cap * task.priority * dist_factor

    def submit_bid(self, task):
        # Re-check status before bidding
        if task.task_id in self.assigned_tasks or self.my_active_task:
            return

        my_score = self.calculate_score(task)
        
        # Publish Bid
        bid_msg = Bid()
        bid_msg.task_id = task.task_id
        bid_msg.robot_id = self.robot_id
        bid_msg.score = my_score
        self.bid_pub.publish(bid_msg)
        
        # Log own bid
        if task.task_id not in self.bids_received:
            self.bids_received[task.task_id] = {}
        self.bids_received[task.task_id][self.robot_id] = my_score

        self.get_logger().info(f"Bid {my_score:.2f} on Task {task.task_id}")
        
        def auction_timer():
            self._auction_timer.cancel()
            self.check_auction_result(task)

        self._auction_timer = self.create_timer(2.0, auction_timer)

    def check_auction_result(self, task):
        # Task already assigned and announced
        if task.task_id in self.assigned_tasks:
            return

        all_bids = self.bids_received.get(task.task_id, {})
        my_bid = all_bids.get(self.robot_id, 0.0)
        
        if my_bid == 0:
            return

        # Find highest bid
        highest_score = -1.0
        winner_id = ""
        
        for rid, score in all_bids.items():
            if score > highest_score:
                highest_score = score
                winner_id = rid
            elif score == highest_score:
                # Tie-breaker: Alphabetical Order
                if rid < winner_id:
                    winner_id = rid

        if winner_id == self.robot_id:
            self.get_logger().info(f"I WON Task {task.task_id}!")
            self.claim_task(task)
        else:
            self.get_logger().info(f"I LOST Task {task.task_id} to {winner_id}")

    def claim_task(self, task):
        # Broadcast the Allocation
        msg = TaskAllocation()
        msg.task_id = task.task_id
        msg.robot_id = self.robot_id
        msg.status = "ASSIGNED"
        self.alloc_pub.publish(msg)
        
        # Update Internal State
        self.my_active_task = task
        self.assigned_tasks[task.task_id] = self.robot_id
        
        # Execution (placeholder)
        # TODO: implement task execution logic
        self.get_logger().info(f"EXECUTION STARTED: Driving to {task.location.x}, {task.location.y}")

def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocator()
    rclpy.spin(node)
    rclpy.shutdown()
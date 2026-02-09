import rclpy
from rclpy.node import Node
from cares_interfaces.msg import Task, Bid, TaskAllocation, SwarmStatus
from geometry_msgs.msg import Point
from std_msgs.msg import String
import random

class TaskAllocator(Node):
    def __init__(self):
        super().__init__('task_allocator')

        # Parameters
        self.declare_parameter('robot_id', 'tb3_0')
        self.robot_id = self.get_parameter('robot_id').value

        # State
        self.my_capabilities = {}
        self.swarm_knowledge = {}
        self.current_pose = Point()
        self.bids_received = {} 
        self.assigned_tasks = {}
        self.my_active_task = None
        self.tasks_bid_on = set()
        self.pending_auction_tasks = set()
        self._bid_timers = {}
        self._auction_timers = {}

        # Subscribers
        self.create_subscription(SwarmStatus, '/swarm/status', self.cap_callback, 10)
        self.create_subscription(Task, '/mission/tasks', self.task_callback, 10)
        self.create_subscription(Bid, '/swarm/bids', self.bid_listen_callback, 10)
        self.create_subscription(TaskAllocation, '/swarm/allocation', self.allocation_callback, 10)

        # Publishers
        self.bid_pub = self.create_publisher(Bid, '/swarm/bids', 10)
        self.log_pub = self.create_publisher(String, '/swarm/logs', 10)
        self.task_pub = self.create_publisher(Task, '/mission/tasks', 10)
        self.alloc_pub = self.create_publisher(TaskAllocation, '/swarm/allocation', 10)

        # Timers
        self.create_timer(2.0, self.health_check)


    # --- CALLBACKS ---
    def cap_callback(self, msg):
        if msg.robot_id not in self.swarm_knowledge:
            self.swarm_knowledge[msg.robot_id] = {}

        for cap in msg.capabilities:
            self.swarm_knowledge[msg.robot_id][cap.type] = cap.value

        if msg.robot_id == self.robot_id:
            self.my_capabilities = self.swarm_knowledge[self.robot_id]

 
    def allocation_callback(self, msg):
        if msg.status == "ABORTED":
            self.assigned_tasks.pop(msg.task_id, None)
            self.tasks_bid_on.discard(msg.task_id)
            self.pending_auction_tasks.discard(msg.task_id)
            self.bids_received.pop(msg.task_id, None)
            return

        self.assigned_tasks[msg.task_id] = msg.robot_id
        self.pending_auction_tasks.discard(msg.task_id)

        if msg.robot_id == self.robot_id and msg.status == "COMPLETED":
            self.my_active_task = None
            self.publish_log(f"I finished Task {msg.task_id}")


    def bid_listen_callback(self, msg):
        if msg.task_id not in self.bids_received:
            self.bids_received[msg.task_id] = {}
        self.bids_received[msg.task_id][msg.robot_id] = msg.score


    def task_callback(self, task):
        self.get_logger().info(f"Task received: {task.task_id}")

        # Task already assigned?
        if task.task_id in self.assigned_tasks:
            return

        # Already bid on this task?
        if task.task_id in self.tasks_bid_on:
            return

        # Already busy with another task?
        if self.my_active_task is not None:
            return

        # Not capable?
        if not self.check_eligibility(task):
            return

        self.tasks_bid_on.add(task.task_id)

        delay = random.uniform(0.1, 0.5)
        self.get_logger().info(f"Preparing bid for {task.task_id} in {delay:.2f}s")

        def on_bid_timer():
            timer = self._bid_timers.pop(task.task_id, None)
            if timer:
                timer.cancel()
            self.submit_bid(task)

        self._bid_timers[task.task_id] = self.create_timer(delay, on_bid_timer)


    def health_check(self):
        if self.my_active_task is None:
            return
        
        task = self.my_active_task

        for req in task.required_capabilities:
            current_score = self.my_capabilities.get(req, 0.0)
            threshold = task.min_capability_score * 0.8

            if current_score < threshold:
                self.publish_log(
                    f"ABORTING {task.task_id}: "
                    f"{req} dropped to {current_score:.2f} "
                    f"(threshold {threshold:.2f})"
                )
                self.abort_task(task)
                return
    
    # Helper function
    def publish_log(self, message: str):
        self.get_logger().info(message)
        msg = String()
        msg.data = f"[{self.robot_id}] {message}"
        self.log_pub.publish(msg)


    # --- BIDDING LOGIC ---
    def check_eligibility(self, task):
        for req in task.required_capabilities:
            my_score = self.my_capabilities.get(req, 0.0)
            if my_score < task.min_capability_score:
                self.publish_log(
                    f"Ineligible for {task.task_id}: "
                    f"{req} score {my_score:.2f} < required {task.min_capability_score:.2f}"
                )
                return False
        return True


    def calculate_score(self, task):
        cap_sum = sum(self.my_capabilities.get(req, 0.0) for req in task.required_capabilities)
        avg_cap = cap_sum / max(1, len(task.required_capabilities))

        dist = abs(task.location.x - self.current_pose.x) + abs(task.location.y - self.current_pose.y)
        dist_factor = 1.0 / (1.0 + dist)

        priority_norm = task.priority / 10

        return avg_cap * priority_norm * dist_factor


    def submit_bid(self, task):
        # Check if task was already claimed
        if task.task_id in self.assigned_tasks:
            return

        # Check if robot already busy
        if self.my_active_task is not None:
            return

        my_score = self.calculate_score(task)

        bid_msg = Bid()
        bid_msg.task_id = task.task_id
        bid_msg.robot_id = self.robot_id
        bid_msg.score = my_score
        self.bid_pub.publish(bid_msg)

        # Record local bid
        if task.task_id not in self.bids_received:
            self.bids_received[task.task_id] = {}
        self.bids_received[task.task_id][self.robot_id] = my_score

        self.publish_log(f"Bid {my_score:.2f} on Task {task.task_id}")

        self.pending_auction_tasks.add(task.task_id)

        def on_auction_timer():
            timer = self._auction_timers.pop(task.task_id, None)
            if timer:
                timer.cancel()
            self.check_auction_result(task)

        self._auction_timers[task.task_id] = self.create_timer(2.0, on_auction_timer)

    def check_auction_result(self, task):
        # If task already assigned
        if task.task_id not in self.pending_auction_tasks:
            return

        # Task already taken by self (guard)
        if task.task_id in self.assigned_tasks:
            self.pending_auction_tasks.discard(task.task_id)
            return

        all_bids = self.bids_received.get(task.task_id, {})
        my_bid = all_bids.get(self.robot_id, 0.0)

        # Bid should never be zero
        if my_bid == 0:
            self.publish_log(f"Bid should never be zero, check mission requirements ")
            self.pending_auction_tasks.discard(task.task_id)
            return

        # winner: highest score, alphabetical
        highest_score = -1.0
        winner_id = ""

        for rid, score in all_bids.items():
            if score > highest_score:
                highest_score = score
                winner_id = rid
            elif score == highest_score and rid < winner_id:
                winner_id = rid

        self.pending_auction_tasks.discard(task.task_id)

        if winner_id == self.robot_id:
            self.publish_log(f"I WON Task {task.task_id}!")
            self.claim_task(task)
        elif winner_id:
            winner_caps = self.swarm_knowledge.get(winner_id, {})
            primary_req = task.required_capabilities[0] if task.required_capabilities else "GENERAL"
            winner_score = winner_caps.get(primary_req, 0.0)
            my_score = self.my_capabilities.get(primary_req, 0.0)
            self.publish_log(
                f"I LOST Task {task.task_id} to {winner_id}. "
                f"Reason: Their {primary_req} ({winner_score:.2f}) > Mine ({my_score:.2f})"
            )
        else:
            self.publish_log(f"Task {task.task_id} could not be assigned")

    def claim_task(self, task):
        # Broadcast allocation.
        msg = TaskAllocation()
        msg.task_id = task.task_id
        msg.robot_id = self.robot_id
        msg.status = "ASSIGNED"
        self.alloc_pub.publish(msg)

        # Update internal state.
        self.my_active_task = task
        self.assigned_tasks[task.task_id] = self.robot_id

        self.publish_log(f"EXECUTION STARTED: Driving to {task.location.x}, {task.location.y}")
        # TODO: implement task execution logic


    def abort_task(self, task):
        # Abort allocation
        msg = TaskAllocation()
        msg.task_id = task.task_id
        msg.robot_id = self.robot_id
        msg.status = "ABORTED"
        self.alloc_pub.publish(msg)

        # Clear internal state
        self.my_active_task = None
        self.assigned_tasks.pop(task.task_id, None)
        self.tasks_bid_on.discard(task.task_id)
        self.pending_auction_tasks.discard(task.task_id)
        self.bids_received.pop(task.task_id, None)

        # Re-publish task
        self.publish_log(f"Re-auctioning {task.task_id}")
        self.task_pub.publish(task)


def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocator()
    rclpy.spin(node)
    rclpy.shutdown()
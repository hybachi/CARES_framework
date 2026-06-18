"""
task_allocator.py
Distributed bidding, task assignment, and preemption for a single robot.

Author: H. A. Sharif
Year: 2026
"""

import rclpy
import random
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from chroma_interfaces.msg import Task, Bid, TaskAllocation, SwarmStatus
from chroma_interfaces.action import ExecuteMission


class TaskAllocator(Node):
    """
    Evaluates broadcasted tasks, submits capability-based bids, and handles 
    graceful hand-offs during mid-mission hardware failures.
    """

    def __init__(self):
        super().__init__('task_allocator')
        self.load_params()
        self.setup_state()
        self.setup_pubs_subs()
        self.setup_timers()
        self.get_logger().info(f"[{self.robot_id}] Task Allocator ready.")

    # -------- Setup --------

    def load_params(self):
        self.declare_parameter('robot_id', 'tb3_0')
        self.robot_id = self.get_parameter('robot_id').value

    def setup_state(self):
        self.my_caps = {}
        self.my_degraded_states = {}
        self.swarm_knowledge = {}
        self.current_pose = Point()

        self.bids_received = {}
        self.tasks_bid_on = set()
        self.pending_auctions = set()
        self.bid_timers = {}
        self.auction_timers = {}
        self.preempt_timers = {}

        self.active_task = None
        self.mission_goal_handle = None
        self.assigned_tasks = {}

        self.is_preempting = False
        self.preempting_tasks = set()

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

        # Delegate local execution to action server
        self.mission_client = ActionClient(self, ExecuteMission, f'/{self.robot_id}/execute_mission')

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
                self.my_degraded_states[cap.type] = cap.is_degraded

        if msg.robot_id == self.robot_id:
            self.my_caps = self.swarm_knowledge[self.robot_id]

    def bid_cb(self, msg):
        if msg.task_id not in self.bids_received:
            self.bids_received[msg.task_id] = {}
        self.bids_received[msg.task_id][msg.robot_id] = msg.score

    def alloc_cb(self, msg):
        if msg.status == "CANCELLED":
            self.clear_task_state(msg.task_id)
            if self.active_task and self.active_task.task_id == msg.task_id:
                self.halt_execution()
            return

        if msg.status == "PREEMPT_REQUEST":
            self.preempting_tasks.add(msg.task_id)
            self.clear_auction_state(msg.task_id)
            if msg.robot_id != self.robot_id:
                self.assigned_tasks.pop(msg.task_id, None)
            return

        if msg.status == "ABORTED":
            if self.assigned_tasks.get(msg.task_id) == msg.robot_id:
                self.clear_task_state(msg.task_id)
            return

        if msg.status == "ASSIGNED":
            self.assigned_tasks[msg.task_id] = msg.robot_id
            self.pending_auctions.discard(msg.task_id)
            self.preempting_tasks.discard(msg.task_id)

            # Ensure handoff completes before halting work
            if self.active_task and self.active_task.task_id == msg.task_id and self.is_preempting:
                if msg.robot_id != self.robot_id:
                    self.publish_log(f"Handoff verified. Yielding {msg.task_id} to {msg.robot_id}.")
                    self.halt_execution()
                else:
                    self.publish_log(f"No healthier peer found. Continuing {msg.task_id}.")
                    self.is_preempting = False

        if msg.status == "COMPLETED" and msg.robot_id == self.robot_id:
            self.active_task = None
            self.is_preempting = False
            self.mission_goal_handle = None

    def task_cb(self, task):
        self.get_logger().info(f"Received Task {task.task_id}.")

        # Allow interventions during active preemption requests
        is_rescue = task.task_id in self.preempting_tasks

        if not is_rescue and task.task_id in self.assigned_tasks:
            return
        if task.task_id in self.tasks_bid_on or task.task_id in self.pending_auctions:
            return
        if self.active_task is not None and self.active_task.task_id != task.task_id:
            return

        reqs = list(task.required_capabilities)
        if "BATTERY" not in reqs:
            reqs.append("BATTERY")

        if not self.check_eligibility(task.task_id, reqs, task.min_capability_score, is_rescue):
            return

        self.tasks_bid_on.add(task.task_id)

        # Prevent network collisions during swarm broadcasts
        delay = random.uniform(0.1, 0.5)
        self.schedule_timer(self.bid_timers, task.task_id, delay, lambda: self.submit_bid(task, reqs))

    # -------- Action Client Callbacks --------

    def goal_accepted_cb(self, future):
        self.mission_goal_handle = future.result()

        if not self.mission_goal_handle.accepted:
            self.get_logger().error("Mission Executor rejected the goal.")
            self.active_task = None
            return

        result_future = self.mission_goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        result = future.result().result
        status = future.result().status

        # Handle explicit cancellations or action failures
        if status == 5 or not result.success:
            if self.active_task is not None:
                self.publish_log(f"Execution failed. Re-auctioning {self.active_task.task_id}.")
                self.publish_allocation(self.active_task.task_id, "ABORTED")
                self.task_pub.publish(self.active_task)
        else:
            self.publish_log(f"{self.active_task.task_id} COMPLETED.")
            self.publish_allocation(self.active_task.task_id, "COMPLETED")

        if self.active_task:
            self.bids_received.pop(self.active_task.task_id, None)

        self.active_task = None
        self.mission_goal_handle = None

    # -------- Core Logic --------

    def health_check(self):
        if self.active_task is None or self.is_preempting:
            return

        task = self.active_task
        reqs = list(task.required_capabilities)
        if "BATTERY" not in reqs:
            reqs.append("BATTERY")

        for req in reqs:
            if self.my_degraded_states.get(req, False):
                self.publish_log(f"Degradation in {req}. Initiating preemption for {task.task_id}.")
                self.request_preemption(task)
                return

    def request_preemption(self, task):
        self.is_preempting = True
        self.publish_allocation(task.task_id, "PREEMPT_REQUEST")

        # Prevent old scores from skewing re-auctions
        self.bids_received.pop(task.task_id, None)

        self.publish_log("Preemption requested. Continuing execution until handoff.")
        self.schedule_timer(self.preempt_timers, task.task_id, 0.5, lambda: self.task_pub.publish(task))

    def check_eligibility(self, task_id, reqs, min_score, is_rescue=False):
        min_req = min_score if min_score > 0.0 else 0.4

        # Prevent total failure during emergency handoffs
        if is_rescue:
            min_req *= 0.5

        for req in reqs:
            my_score = self.my_caps.get(req, 0.0)

            if my_score <= 0.01:
                self.publish_log(f"Rejected {task_id}: {req} critically failed ({my_score:.2f}).")
                return False

            if my_score < min_req:
                self.publish_log(f"Rejected {task_id}: {req} too low ({my_score:.2f} < {min_req:.2f}).")
                return False

            if self.my_degraded_states.get(req, False):
                if not is_rescue:
                    self.publish_log(f"Rejected {task_id}: {req} is degraded.")
                    return False
                else:
                    self.publish_log(f"Warning: Bidding on {task_id} despite degraded {req} (rescue).")

        return True

    def calculate_score(self, task, reqs):
        avg_cap = sum(self.my_caps.get(r, 0.0) for r in reqs) / max(1, len(reqs))
        dist = abs(task.location.x - self.current_pose.x) + abs(task.location.y - self.current_pose.y)
        score = float(avg_cap * (1.0 / (1.0 + dist * 0.2)))

        # Ensure healthier peers win degraded auctions
        for req in reqs:
            if self.my_degraded_states.get(req, False):
                return score * 0.25

        return score

    def submit_bid(self, task, reqs):
        is_my_rescue = self.is_preempting and self.active_task and self.active_task.task_id == task.task_id

        if self.active_task is not None and not is_my_rescue:
            return
        if not is_my_rescue:
            if task.task_id in self.pending_auctions or task.task_id in self.assigned_tasks:
                return

        my_score = self.calculate_score(task, reqs)

        bid_msg = Bid()
        bid_msg.task_id = task.task_id
        bid_msg.robot_id = self.robot_id
        bid_msg.score = my_score
        self.bid_pub.publish(bid_msg)

        if task.task_id not in self.bids_received:
            self.bids_received[task.task_id] = {}
        self.bids_received[task.task_id][self.robot_id] = my_score

        self.pending_auctions.add(task.task_id)
        self.publish_log(f"Bid {my_score:.2f} on {task.task_id}.")
        self.schedule_timer(self.auction_timers, task.task_id, 2.0, lambda: self.resolve_auction(task))

    def resolve_auction(self, task):
        if task.task_id not in self.pending_auctions:
            return

        is_my_rescue = self.is_preempting and self.active_task and self.active_task.task_id == task.task_id

        if not is_my_rescue and task.task_id in self.assigned_tasks:
            self.pending_auctions.discard(task.task_id)
            return

        all_bids = self.bids_received.get(task.task_id, {})
        winner_id, top_score = "", -1.0

        for robot_id, score in all_bids.items():
            if score > top_score or (score == top_score and robot_id < winner_id):
                top_score, winner_id = score, robot_id

        self.pending_auctions.discard(task.task_id)
        self.tasks_bid_on.discard(task.task_id)

        if winner_id == self.robot_id:
            if is_my_rescue:
                self.publish_log("No healthier peer found. Pushing through degradation.")
                self.is_preempting = False
            else:
                self.publish_log(f"Won Task {task.task_id}.")
                self.claim_task(task)

        elif winner_id:
            self.publish_log(f"Lost Task {task.task_id} to {winner_id}.")
            if is_my_rescue:
                self.publish_log(f"Handoff verified locally. Yielding {task.task_id} to {winner_id}.")
                self.yield_preempted_task()

    def claim_task(self, task):
        self.active_task = task
        self.is_preempting = False
        self.assigned_tasks[task.task_id] = self.robot_id

        self.publish_allocation(task.task_id, "ASSIGNED")

        goal = ExecuteMission.Goal()
        goal.task = task
        self.mission_client.wait_for_server()

        future = self.mission_client.send_goal_async(goal)
        future.add_done_callback(self.goal_accepted_cb)

    # -------- State Management --------

    def clear_auction_state(self, task_id):
        self.tasks_bid_on.discard(task_id)
        self.pending_auctions.discard(task_id)
        self.bids_received.pop(task_id, None)
        self.destroy_timer_safely(self.bid_timers, task_id)
        self.destroy_timer_safely(self.auction_timers, task_id)

    def clear_task_state(self, task_id):
        self.clear_auction_state(task_id)
        self.assigned_tasks.pop(task_id, None)
        self.preempting_tasks.discard(task_id)

    def halt_execution(self):
        if self.mission_goal_handle:
            self.mission_goal_handle.cancel_goal_async()
            self.mission_goal_handle = None

        self.active_task = None
        self.is_preempting = False

    def yield_preempted_task(self):
        # Clean up local state after handoff
        if self.mission_goal_handle:
            self.mission_goal_handle.cancel_goal_async()
            self.mission_goal_handle = None

        if self.active_task:
            self.clear_task_state(self.active_task.task_id)

        self.active_task = None
        self.is_preempting = False

    # -------- Timer Scheduling --------

    def schedule_timer(self, timer_dict, task_id, delay, callback):
        self.destroy_timer_safely(timer_dict, task_id)

        def on_fire():
            self.destroy_timer_safely(timer_dict, task_id)
            callback()

        timer_dict[task_id] = self.create_timer(delay, on_fire)

    def destroy_timer_safely(self, timer_dict, task_id):
        timer = timer_dict.pop(task_id, None)
        if timer:
            try:
                if not timer.is_canceled():
                    timer.cancel()
                self.destroy_timer(timer)
            except Exception:
                # Safely ignore already destroyed handles
                pass

    # -------- Helpers --------

    def publish_allocation(self, task_id, status):
        msg = TaskAllocation()
        msg.task_id = task_id
        msg.robot_id = self.robot_id
        msg.status = status
        self.alloc_pub.publish(msg)

    def publish_log(self, message):
        self.get_logger().info(message)
        msg = String()
        msg.data = f"[{self.robot_id}] {message}"
        self.log_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
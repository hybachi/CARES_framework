# Week 3 Progress Log

## Objectives

The primary objective for Week 3 was to implement **Decentralized Coordination**. This involved moving the swarm from passive capability reporting to active, autonomous decision-making. The goal was to develop a market-based Task Allocation (MRTA) system where robots self-select tasks based on their capability scores without a central coordinator.

## Completed Tasks

**1. Coordination Interfaces**

- Defined standard ROS2 messages required for the contract protocol:
  - `Task.msg`: encodes mission requirements
  - `Bid.msg`: allows robots to broadcast their bids
  - `TaskAllocation.msg`: a distributed ledger confirming robot task allocation

**2. Task Allocator Node**

- Developed the `task_allocator.py` node that runs on every robot for decision-making.
- The allocator subscribes to `/swarm/status` and utilises capability-aware bidding as follows: $$ Score = \frac{AvgCapability \times TaskPriority}{1 + Distance} $$
- It uses a Parallel Auction strategy rather than SSI for better asynchronous bidding.
- A Randomized Timer (0.1s - 0.5s) for bidding is used to reduce network packet collisions.

**3. Mission Spawner Utility**

- Created a `mission_spawner.py` to publish dummy tasks for testing.
- Verified that different tasks are correctly routed to the appropriate robots based on capability definitions.

## Current Project State

At the end of week 3:

- The swarm acts as a decentralized market.
- Robots autonomously accept or reject tasks based on their current health and location.

## Next Steps

- Create a GUI dashboard to visualise swarm status and assign missions.
- Implement Resilience, where if a robot degrades during a task it is forced to abort the task so others can take over.

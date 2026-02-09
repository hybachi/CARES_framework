# Week 4 Progress Log

## Objectives

The primary objective for Week 4 was to implement the **Human-Swarm Interaction (HSI)** dashboard along with a layer of resilience. While the swarm could allocate tasks, it lacked the ability to handle failures during execution.

## Completed Tasks

**Execution Monitoring**

- Integrated a continuous `health_check` loop in the `task_allocator` node.
- Implemented the Abort Protocol:
  - If a robot's capability score drops below the task threshold during execution, the robot triggers `abort_task()`.
  - This broadcasts a `TaskAllocation` message with status `ABORTED` and the task is re-auctioned.

**HSI Dashboard**

- Developed a standalone web-based dashboard using NiceGUI and a custom `RosBridge` node.
- Implemented three core pages:
  - Overview: 3D visualization (placeholder) and high-level metrics.
  - Fleet Status: Detailed cards for each robot showing capability history and current status (requires improvements).
  - Missions: An interface to dispatch tasks and monitor auction logs as well as task allocations.

## Current Project State

At the end of week 4:

- The system is observable through a dashboard interface.
- Robots autonomously reject work they can no longer perform and the swarm re-allocates it.

## Next Steps

- Implement heterogeneity, sensor capabilities and robot types into `capability_manager`.
- Improve Fleet Status page for more robot information.

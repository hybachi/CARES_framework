# Week 2 Progress Log

## Objectives

The primary objective for Week 2 was to implement **Capability Awareness**. This involved defining the data structures for robot capabilities and creating the distributed software architecture that allows robots to continuously broadcast their operational state (health, battery, connectivity) to the swarm.

## Completed Tasks

**1. Interface Definition**

- Defined the core ROS2 custom interfaces `cares_interfaces` to standardize communication:
  - `Capability.msg` encodes dynamic attributes (type, normalized value, confidence score).
  - `SwarmStatus.msg` holds a vector of capabilities attached to a specific Robot ID.

**2. Capability Manager Node**

- Developed the `capability_manager` node in Python, which acts as the middleware between hardware and the swarm intelligence layer.
- Implemented logic to convert raw telemetry into standardized capability scores.
- Added simulated fault injections to visualise degradation over time.

**3. QoS Profiles**

- Configured ROS2 Quality of Service (QoS) profiles to use `RELIABLE` and `TRANSIENT_LOCAL`.
- A robot that connects to the topic immediately receives the last known status of the swarm, ensuring persistent situational awareness.

## Current Project State

At the end of week 2:

- The Capability Awareness framework is functional. Robots are no longer static entities and can dynamically report their state.

## Next Steps

- Implement Decentralized Coordination by connecting capability scores to bidding logic.
- Develop a market-based Task Allocator node.

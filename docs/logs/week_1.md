# Week 1 Progress Log

## Objectives

The primary objective for Week 1 was to establish a reliable simulation environment using ROS2 Humble and Gazebo Fortress for single robot and multi-robot configurations.

## Completed Tasks

**1. Simulation Environment Setup**

- Successfully configured Gazebo Fortress (Ignition) with ROS2 Humble and bridged relevant topics between both.

**2. Robot Description (Xacro)**

- Refactored the original Turtlebot3 URDF into modular Xacro components:
  - Core kinematic structure
  - Gazebo control plugins
  - Gazebo sensor plugins
- This makes it easier to scale the URDF for simulation and physical environments.

**3. Multi-Robot Simulation**

- Implemented a multi-robot Turtlebot3 simulation with:
  - Independent control and sensors
  - Shared global TF tree with robot frame prefixes

## Current Project State

At the end of week 1:

- Multiple Turtlebot3 robots can be spawned reliably in simulation and controlled independently.

## Next Steps

- Establish the core Capability Awareness framework, robots should be able to discover static capabilities on demand.
- Simulate the JetHexa in Gazebo for true heterogeneity.

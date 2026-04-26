# CARES Framework

![alt text](https://img.shields.io/badge/ROS_2-Humble-22314E?style=flat&logo=ros)
![alt text](https://img.shields.io/badge/Status-Active_Development-success)

**Capability-Aware Resilient Emergency Swarms (CARES)** is a thesis project focused on the design and implementation of a capability-aware, decentralized, and resilient heterogeneous swarm framework for **Urban Search and Rescue (USAR)** scenarios.

In high-stakes USAR missions, robots frequently face environmental hazards, structural occlusions, battery drain, and hardware failures. Traditional multi-robot systems rely on static assumptions about what a robot can do. CARES introduces a **Dynamic Capability Model**, enabling heterogeneous robots (wheeled, legged, aerial) to continuously evaluate and broadcast their real-time operational health. This allows the swarm to autonomously coordinate, adapt to partial failures, and reallocate tasks on the fly without relying on a centralized controller.

## Tech Stack & Hardware

- **Middleware**: ROS 2 Humble
- **Simulator**: Gazebo / Ignition Fortress
- **UI/Dashboard**: NiceGUI (Python)
- **Hardware Platforms**:
  - Turtlebot3 Burger
  - HiWonder JetHexa

## Installation & Setup (Pixi)

This project uses [Pixi](https://pixi.sh) and [Robostack](https://robostack.github.io) to manage its ROS 2 environment. This ensures all dependencies (including ROS 2 Humble) are installed in an isolated project folder, avoiding conflicts with the system.

### Install Pixi

If you don't have Pixi installed, run:

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

Then check if Pixi installed successfully:

```bash
pixi --version
```

### Install Project Dependencies

Navigate to workspace folder and install all required packages

```bash
pixi install
```

_Note: This will download ROS 2 and all dependencies into the `.pixi/` folder._

### Build the Workspace

Always run commands using the `pixi run` prefix to ensure the virtual environment is used

```bash
pixi run colcon build --symlink-install
```

> **Warning**: Do NOT source `/opt/ros/humble/setup.bash` while using this project. Doing so will cause environment leakage and build errors. Please remove it from `.bashrc` before building.

### Shell Activation

If you prefer a standard terminal workflow where you don't have to type `pixi run` every time, you can enter the isolated shell

```bash
pixi shell
# You are now inside the environment
ros2 launch ...
```

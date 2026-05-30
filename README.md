# CHROMA Framework

![alt text](https://img.shields.io/badge/ROS_2-Humble-22314E?style=flat&logo=ros)
![alt text](https://img.shields.io/badge/Status-Active_Development-success)

**Capability-aware Heterogeneous Robot Operations and Mission Allocation (CHROMA)** is a thesis project focused on the design and implementation of a capability-aware, decentralized, and resilient heterogeneous swarm framework for **Urban Search and Rescue (USAR)** scenarios.

In high-stakes USAR missions, robots frequently face environmental hazards, structural occlusions, battery drain, and hardware failures. Traditional multi-robot systems rely on static assumptions about what a robot can do. CHROMA introduces a **Dynamic Capability Model**, enabling heterogeneous robots (wheeled, legged, aerial) to continuously evaluate and broadcast their real-time operational health. This allows the swarm to autonomously coordinate, adapt to partial failures, and reallocate tasks on the fly without relying on a centralized controller.

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

## Configuration

Swarm compositions and robot capabilities are defined entirely via YAML files located in the `chroma_bringup/config/` directory.

### Robot Profiles

A **Robot Profile** defines the genetic makeup of a robot type (e.g., a TurtleBot3 or a JetHexa). It defines base capabilities and operational thresholds (to be extended later).

Example: `chroma_bringup/config/tb3_profile.yaml`

```yaml
/**:
  ros__parameters:
    robot_type: "UGV_WHEELED"
    model_name: "turtlebot3_burger"
    
    # Base Capabilities (0.0 to 1.0)
    capabilities:
      MOBILITY: 1.0
      VISION: 0.0
      NETWORK: 1.0
      BATTERY: 1.0

    # Degradation Thresholds (Triggers re-allocation)
    thresholds:
      MOBILITY: 0.3
      VISION: 0.4
      BATTERY: 0.15

```

### Swarm Configuration

The `swarm_config.yaml` file located in `chroma_simulation/config/` directory is used to generate the simulation fleet. Edit this file to add or remove robots from the simulation.

Example: 

```yaml
swarm:
  tb3_0:
    config: "tb3_profile.yaml"
    urdf: "turtlebot3/robot.urdf.xacro"
    x: 0.0
    y: 0.0
    yaw: 0.0
    
  tb3_1:
    config: "tb3_profile.yaml"
    urdf: "turtlebot3/robot.urdf.xacro"
    x: 1.0
    y: 0.0
    yaw: 0.0
    
  jethexa_0:
    config: "jethexa_profile.yaml"
    urdf: "jethexa/robot.urdf.xacro"
    x: 0.0
    y: 1.0
    yaw: 0.0

```

> **Note**: The launch file will search for config files in `chroma_bringup/config/` and urdf files inside `chroma_simulation/urdf/`.

## Running the Simulation

Once the configuration is set, launching the swarm simulation in Gazebo and running the HSI dashboard uses the following commands.

#### Launch the Simulation

```bash
pixi run sim
```

#### Start the Dashboard

```bash
pixi run dash
```
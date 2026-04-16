# Pegasus Isaac

A ROS 2 package where we define the launch files for starting a simulation using Isaac Sim and Pegasus Simulator.

## Installation

### Prerequisites

- [ROS 2](https://index.ros.org/doc/ros2/Installation/)
- [Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html)
- [Pegasus Simulator](https://pegasussimulator.github.io/PegasusSimulator/)

### Build

```bash
mkdir -p ~/pegasus_ws/src
cd ~/pegasus_ws/src

# Clone the pegasus isaac package
git clone https://github.com/PegasusResearch/pegasus_isaac

# Build the workspace
cd ~/pegasus_ws
colcon build --symlink-install
```

# Developer Team 

* [Marcelo Jacinto](https://github.com/MarceloJacinto)
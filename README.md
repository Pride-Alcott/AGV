# Solar Farm AGV - ROS 2 Workspace

This repository contains the ROS 2 (Humble) workspace for the Solar Farm Autonomous Guided Vehicle (AGV), including MAVROS integration for Pixhawk control, sensor fusion, and mission logic.

## Structure
- `src/` - Source code and ROS 2 packages
- `build/`, `install/`, `log/` - Ignored in Git

## Usage
```bash
cd AGV
colcon build
source install/setup.bash

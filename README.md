# UR5e Experiments

ROS2 workspace for Universal Robots manipulation and control experiments.

## Overview

This workspace contains packages for working with the Universal Robots UR5e manipulator, including robot description files, visualization tools, and custom control implementations.

## Package Structure

### `ur_description`
Official Universal Robots ROS2 description package providing:
- URDF/XACRO models for all UR robot series (UR3, UR3e, UR5, UR5e, UR10, UR10e, UR16e, UR20, UR30)
- High-fidelity visual and collision meshes
- Accurate kinematic parameters and joint limits
- ros2_control integration macros

**Source:** https://github.com/UniversalRobots/Universal_Robots_ROS2_Description

### `ur5e_control`
Custom control package for UR5e development:
- Example motion generation nodes
- Test utilities for joint control
- Development framework for custom behaviors

**Note:** Current implementation publishes directly to `/joint_states` for testing purposes. For production use, integrate with ros2_control framework and use proper action clients.

## Prerequisites

### System Requirements
- Ubuntu 22.04 (Noble) or later
- ROS2 Jazzy
- Python 3.10+

### Required ROS2 Packages
```bash
sudo apt-get update
sudo apt-get install -y \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control
```

## Building the Workspace

### Clone and Build
```bash
# Navigate to workspace
cd /home/anish/data/SpaceTS/robotics/ur5e_experiments

# Build all packages
colcon build --symlink-install

# Source the workspace (bash)
source install/setup.bash

# Source the workspace (zsh)
source install/setup.zsh
```

### Clean Build
```bash
# Remove build artifacts
rm -rf build/ install/ log/

# Rebuild from scratch
colcon build --symlink-install
```

## Usage

### 1. Visualize UR5e in RViz

Launch the UR5e robot model in RViz for visualization:

```bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur5e
```

**Options:**
- `ur_type`: Robot model to load (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30)
- `safety_limits`: Enable safety limits controller (default: true)
- `description_file`: Path to custom URDF file

**Example with different robot:**
```bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur10e
```

### 2. Run Test Motion Controller

Execute the sinusoidal joint motion test:

```bash
# Terminal 1: Launch visualization
ros2 launch ur_description view_ur.launch.py ur_type:=ur5e

# Terminal 2: Run motion controller
ros2 run ur5e_control move_ur5e
```

The robot will move all joints in sinusoidal patterns with phase offsets.

### 3. View Launch File Arguments

Display all available launch arguments:

```bash
ros2 launch ur_description view_ur.launch.py --show-args
```

### 4. Inspect Robot Topics

Monitor joint states being published:

```bash
# List active topics
ros2 topic list

# Echo joint states
ros2 topic echo /joint_states

# View topic info
ros2 topic info /joint_states
```

### 5. Check TF Tree

Visualize the robot's transform tree:

```bash
# Generate TF tree PDF
ros2 run tf2_tools view_frames

# View the generated frames.pdf
evince frames.pdf
```

## Package Details

### ur_description

**Launch Files:**
- `view_ur.launch.py`: Visualize robot model in RViz with joint state publisher

**Key Directories:**
- `urdf/`: Robot URDF and XACRO files
- `meshes/`: Visual and collision mesh files for all UR models
- `config/`: Robot-specific configuration (kinematics, joint limits, physical parameters)
- `rviz/`: RViz configuration files

### ur5e_control

**Nodes:**

#### `move_ur5e`
Generates and publishes sinusoidal joint trajectories for testing.

**Published Topics:**
- `/joint_states` (sensor_msgs/JointState): Joint positions at 20 Hz

**Parameters:**
- Update rate: 20 Hz (0.05s timer)
- Amplitude: 0.5 radians
- Phase offsets: 0 to 5 radians across joints

**Joint Names:**
- shoulder_pan_joint
- shoulder_lift_joint
- elbow_joint
- wrist_1_joint
- wrist_2_joint
- wrist_3_joint

## Development Notes

### Architecture
The workspace follows a modular architecture:
- `ur_description`: Provides robot model and kinematic definitions
- `ur5e_control`: Implements control logic and behaviors

### Current Limitations
- `ur5e_control` currently bypasses ros2_control framework
- Direct `/joint_states` publishing is suitable for visualization testing only
- Production systems should use action clients with `FollowJointTrajectory` interface

### Future Enhancements
- Integration with ros2_control framework
- Gazebo simulation support
- MoveIt2 motion planning integration
- Trajectory action server implementation

## Troubleshooting

### Build Issues

**Problem:** Package not found after build
```bash
# Ensure workspace is sourced
source install/setup.zsh  # or setup.bash

# Verify package is built
ros2 pkg list | grep ur
```

**Problem:** Colcon build warnings about package override
```bash
# This is expected when ur_description exists in both workspace and system
# To suppress, add flag:
colcon build --symlink-install --allow-overriding ur_description
```

### Runtime Issues

**Problem:** RViz not displaying robot
- Verify robot_description parameter is loaded
- Check TF tree is being published
- Ensure meshes are found in package path

**Problem:** Joint states not updating
- Confirm `move_ur5e` node is running
- Check `/joint_states` topic is being published
- Verify joint names match URDF definition

## License

- `ur_description`: BSD-3-Clause (Universal Robots A/S)
- `ur5e_control`: To be determined

## References

- [Universal Robots ROS2 Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
- [ROS2 Control Documentation](https://control.ros.org/)
- [Universal Robots Support](https://www.universal-robots.com/)


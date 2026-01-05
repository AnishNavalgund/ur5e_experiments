# UR5e Experiments

ROS2 workspace for Universal Robots UR5e manipulation experiments.

## Package Overview

### 1. `Universal_Robots_ROS2_Description`
Official Universal Robots ROS2 description package containing URDF models for all UR robots:
- UR3, UR3e, UR5, UR5e, UR10, UR10e, UR16e, UR20, UR30
- Accurate meshes, kinematics, joint limits
- Visual and collision geometries

**Source:** https://github.com/UniversalRobots/Universal_Robots_ROS2_Description

### 2. `Universal_Robots_ROS2_Gazebo_Simulation`
Gazebo simulation setup for Universal Robots:
- Launch files for simulated UR robots
- Integration with ros2_control
- Example worlds and configurations

**Source:** https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation

### 3. `ur5e_control`
Custom test package for UR5e joint control:
- Simple sinusoidal joint trajectory publisher
- Test node for UR5e motion validation

## Quick Start

### 1. Build the Workspace
```bash
cd /home/anish/data/SpaceTS/robotics/ur5e_experiments
colcon build --symlink-install
source install/setup.bash
```

### 2. Run UR5e Test Controller
```bash
# Terminal 1: Launch UR5e in Gazebo (if available)
# ros2 launch ur_simulation_gazebo ur_sim_control.launch.py

# Terminal 2: Run custom joint mover
ros2 run ur5e_control move_ur5e
```

## Dependencies

Install required dependencies:
```bash
sudo apt-get install ros-$ROS_DISTRO-ur-description \
                     ros-$ROS_DISTRO-ur-simulation-gazebo \
                     ros-$ROS_DISTRO-ros2-control \
                     ros-$ROS_DISTRO-ros2-controllers
```

## Package Details

### `ur5e_control` Nodes

**`move_ur5e`**
- **Description:** Publishes sinusoidal joint trajectories for UR5e
- **Topic:** `/joint_states` (sensor_msgs/JointState)
- **Joints:** 6-DOF (shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3)
- **Update Rate:** 20 Hz

## Notes

This workspace is separate from the main satellite simulation project and focuses on:
- Universal Robots manipulation
- UR5e kinematic experiments
- Industrial robot control testing

For satellite robotics (Franka FR3), see the main `space_robotics` workspace.

## License

- `Universal_Robots_ROS2_Description`: BSD-3-Clause (Universal Robots)
- `Universal_Robots_ROS2_Gazebo_Simulation`: BSD-3-Clause (Universal Robots)
- `ur5e_control`: Apache-2.0


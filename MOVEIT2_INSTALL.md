# MoveIt2 Installation from Source

Complete guide for building and installing MoveIt2 (Motion Planning Framework) from source for ROS2 Jazzy.

## Prerequisites

- **ROS2 Jazzy installed** - See [ROS2_INSTALL.md](ROS2_INSTALL.md)
- **ros2_control installed** - See [ROS2_CONTROL_INSTALL.md](ROS2_CONTROL_INSTALL.md)
- **Ubuntu 24.04** (Noble Numbat)
- **8GB+ RAM** (16GB recommended for building)

---

## Overview

MoveIt2 consists of multiple repositories:

1. **moveit2** - Core motion planning framework
2. **moveit_msgs** - MoveIt message definitions
3. **moveit_resources** - Test robots and scenes
4. **moveit_task_constructor** - Task planning (optional)
5. **moveit_visual_tools** - RViz visualization helpers (optional)

---

## 1. Create Workspace

```bash
# Create separate workspace for MoveIt2
mkdir -p ~/moveit2_ws/src
cd ~/moveit2_ws/src
```

**Note:** You can add to existing workspace, but separate is recommended for easier management.

---

## 2. Clone Source Repositories

### Method 1: Using vcstool (Recommended)

```bash
cd ~/moveit2_ws/src

# Clone MoveIt2 main repository
git clone https://github.com/moveit/moveit2.git -b jazzy

# Use MoveIt's repos file to get all dependencies
cd moveit2
vcs import < moveit2.repos --recursive

cd ~/moveit2_ws
```

### Method 2: Manual clone (Core only)

```bash
cd ~/moveit2_ws/src

# Core MoveIt2
git clone https://github.com/moveit/moveit2.git -b jazzy

# MoveIt messages
git clone https://github.com/moveit/moveit_msgs.git -b jazzy

# MoveIt resources (test robots)
git clone https://github.com/moveit/moveit_resources.git -b jazzy

# Geometric shapes library
git clone https://github.com/moveit/geometric_shapes.git -b jazzy

# SRDF (Semantic Robot Description Format)
git clone https://github.com/moveit/srdfdom.git -b jazzy

# Random numbers library
git clone https://github.com/moveit/random_numbers.git -b jazzy
```

### Optional: Visual Tools

```bash
cd ~/moveit2_ws/src

# MoveIt Visual Tools (helpful for debugging)
git clone https://github.com/moveit/moveit_visual_tools.git -b jazzy

# RViz Visual Tools (dependency)
git clone https://github.com/PickNikRobotics/rviz_visual_tools.git -b jazzy
```

### Optional: Task Constructor

```bash
cd ~/moveit2_ws/src

# MoveIt Task Constructor (advanced task planning)
git clone https://github.com/moveit/moveit_task_constructor.git -b jazzy
```

---

## 3. Install Dependencies

```bash
cd ~/moveit2_ws

# Source ROS2 and ros2_control
source /opt/ros/jazzy/setup.bash
# If built from source:
# source ~/ros2_jazzy/install/setup.bash
source ~/ros2_control_ws/install/setup.bash

# Update rosdep
rosdep update

# Install all dependencies
rosdep install --from-paths src --ignore-src -y -r
```

**Key dependencies installed:**
- **OMPL** (Open Motion Planning Library) - Path planning
- **FCL** (Flexible Collision Library) - Collision checking
- **OCTOMAP** - 3D occupancy mapping
- **Eigen3** - Linear algebra
- **Bullet** - Physics and collision detection
- **URDF**, **KDL** - Robot modeling and kinematics
- **warehouse_ros** - Database support

### Install additional planning libraries

```bash
# Install additional motion planners
sudo apt install -y \
  ros-jazzy-pilz-industrial-motion-planner \
  ros-jazzy-chomp-motion-planner
```

---

## 4. Build MoveIt2

```bash
cd ~/moveit2_ws

# Build in Release mode (recommended)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Or parallel build (faster, uses more RAM)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
```

**Build time:** 20-60 minutes depending on CPU and optional packages

### If build fails due to memory

```bash
# Reduce parallel jobs
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2

# Or sequential build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --executor sequential
```

### Build specific packages only

```bash
# Core MoveIt only
colcon build --packages-select \
  moveit_core \
  moveit_ros_planning \
  moveit_ros_planning_interface \
  moveit_planners_ompl

# Setup Assistant only
colcon build --packages-select moveit_setup_assistant

# Servo only (for teleoperation)
colcon build --packages-select moveit_servo
```

---

## 5. Source the Workspace

```bash
cd ~/moveit2_ws
source install/setup.bash
```

### Add to ~/.bashrc

```bash
# Add after ROS2 and ros2_control sourcing
echo "source ~/moveit2_ws/install/setup.bash" >> ~/.bashrc
```

**Important:** Source order:
1. ROS2 (`/opt/ros/jazzy` or `~/ros2_jazzy`)
2. ros2_control (`~/ros2_control_ws`)
3. MoveIt2 (`~/moveit2_ws`)
4. Your project workspace (`~/ros2_ws`)

---

## 6. Verify Installation

### Check MoveIt packages
```bash
ros2 pkg list | grep moveit
```

**Expected output (partial):**
```
moveit_common
moveit_core
moveit_msgs
moveit_planners_ompl
moveit_ros_move_group
moveit_ros_planning
moveit_ros_planning_interface
moveit_ros_visualization
moveit_ros_warehouse
moveit_servo
moveit_setup_assistant
moveit_simple_controller_manager
```

### Test MoveIt Setup Assistant
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

**Expected:** Setup Assistant GUI opens

### Test with demo robot
```bash
ros2 launch moveit2_tutorials demo.launch.py
```

---

## 7. MoveIt2 Package Overview

### Core Packages

#### moveit_core
- **Collision checking** - FCL, Bullet integration
- **Kinematics** - Forward/inverse kinematics (KDL, TracIK, CACHED_IK)
- **Planning scene** - World representation
- **Trajectory processing** - Smoothing, time parameterization
- **Constraint representation** - Position, orientation, joint constraints

#### moveit_ros_planning
- **Planning pipeline** - Adapter plugins for pre/post-processing
- **Planning scene monitor** - Sync with robot state
- **Kinematics plugins** - Plugin interface for IK solvers
- **Constraint samplers** - Sample valid robot states

#### moveit_ros_planning_interface
- **move_group_interface** - C++/Python API for motion planning
- **planning_scene_interface** - Add/remove collision objects
- High-level programming interface (most users interact here)

#### moveit_planners_ompl
- **OMPL planners** - RRT, RRTConnect, PRM, EST, KPIECE, etc.
- **OMPL interface** - Converts MoveIt planning to OMPL format
- Default planner for most applications

#### moveit_ros_move_group
- **move_group node** - Main MoveIt planning and execution node
- **Capabilities** - Plugins for planning, execution, kinematics, TF
- **Action servers** - MoveGroup, ExecuteTrajectory, Pickup, Place

#### moveit_ros_visualization
- **motion_planning_rviz_plugin** - Interactive markers in RViz
- **Planning display** - Visualize trajectories, states
- **Trajectory display** - Animate robot motion

#### moveit_setup_assistant
- **Configuration wizard** - Generate MoveIt config packages
- **SRDF editor** - Define planning groups, end effectors, poses
- **Collision matrix** - Disable self-collision pairs
- **Controllers** - Configure ros2_control integration

#### moveit_servo
- **Real-time servoing** - Cartesian and joint velocity control
- **Teleoperation** - Joystick/spacemouse control
- **Collision avoidance** - Online collision checking
- **Important for this project** - Used in joystick teleoperation

---

### Optional Planners

#### pilz_industrial_motion_planner
- **LIN** - Linear motion in Cartesian space
- **PTP** - Point-to-point joint space motion
- **CIRC** - Circular motion
- Industrial-style motion commands

#### chomp_motion_planner
- **CHOMP** - Covariant Hamiltonian Optimization
- Trajectory optimization with gradient descent
- Good for cluttered environments

---

## 8. Install Additional IK Solvers

### TracIK (Recommended for complex robots)

```bash
cd ~/moveit2_ws/src
git clone https://github.com/traclabs/trac_ik.git -b jazzy

cd ~/moveit2_ws
rosdep install --from-paths src --ignore-src -y -r
colcon build --symlink-install --packages-select trac_ik_kinematics_plugin
```

**Benefits:**
- Faster than KDL
- More reliable convergence
- Handles complex kinematic chains better

### BIO-IK (Alternative)

```bash
cd ~/moveit2_ws/src
git clone https://github.com/PickNikRobotics/bio_ik.git -b jazzy

cd ~/moveit2_ws
rosdep install --from-paths src --ignore-src -y -r
colcon build --symlink-install --packages-select bio_ik
```

---

## 9. Testing MoveIt2

### Test with Panda robot demo

```bash
# Terminal 1: Launch move_group
ros2 launch moveit2_tutorials move_group.launch.py

# Terminal 2: Launch RViz
ros2 launch moveit2_tutorials moveit_rviz.launch.py

# Terminal 3: Run C++ example
ros2 launch moveit2_tutorials move_group_interface_tutorial.launch.py
```

### Test planning pipeline

```bash
# Check available planners
ros2 param get /move_group planning_plugin

# Check planning groups
ros2 service call /move_group/get_planning_scene_world moveit_msgs/srv/GetPlanningScene "{}"
```

---

## 10. Configure for Humanoid Arm

### Generate MoveIt config (if not done)

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

**Steps:**
1. Load URDF: `humanoid_arm_description/urdf/humanoid_arm_5dof_ros2_control.urdf.xacro`
2. Define planning group: "arm" with 5 joints
3. Define end effector: "tcp" (tool center point)
4. Add poses: home, ready, etc.
5. Configure ros2_control controllers
6. Generate package: `humanoid_arm_moveit_config`

### Key configuration files

- **srdf** - Semantic robot description (groups, poses, collision matrix)
- **joint_limits.yaml** - Velocity and acceleration limits
- **kinematics.yaml** - IK solver configuration
- **ompl_planning.yaml** - OMPL planner parameters
- **moveit_controllers.yaml** - ros2_control integration

---

## 11. Build Options

### Debug Build
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### With Tests
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select moveit_core
colcon test-result --verbose
```

### Clean Rebuild
```bash
cd ~/moveit2_ws
rm -rf build install log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## 12. Updating MoveIt2

```bash
cd ~/moveit2_ws

# Pull latest Jazzy changes
cd src/moveit2 && git pull origin jazzy
cd ../..

# Or use vcs
vcs pull src

# Reinstall dependencies
rosdep install --from-paths src --ignore-src -y -r

# Rebuild
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## Troubleshooting

### Issue: "Could not find moveit_core"

**Solution:**
```bash
# Ensure MoveIt2 is built and sourced
cd ~/moveit2_ws
colcon build --packages-select moveit_core
source install/setup.bash
```

### Issue: IK solver fails or is slow

**Solution:**
```bash
# Install TracIK (faster, more robust)
cd ~/moveit2_ws/src
git clone https://github.com/traclabs/trac_ik.git -b jazzy
cd ~/moveit2_ws
colcon build --packages-select trac_ik_kinematics_plugin

# Update kinematics.yaml to use trac_ik
# kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
```

### Issue: Planning fails with "unable to sample valid states"

**Solution:**
```bash
# Check joint limits in URDF match SRDF
# Increase planning time in move_group parameters
# Simplify collision geometry
# Check for joint limit violations
```

### Issue: Setup Assistant won't load URDF

**Solution:**
```bash
# Ensure xacro is processed
xacro humanoid_arm_5dof_ros2_control.urdf.xacro > test.urdf

# Validate URDF
check_urdf test.urdf

# Check for missing meshes
ls humanoid_arm_description/meshes/
```

### Issue: move_group crashes or high CPU usage

**Solution:**
```bash
# Reduce planning scene update rate
# Disable collision checking for distant objects
# Use simplified collision meshes (not visual meshes)
# Increase planning time limit
```

### Issue: Trajectory execution fails

**Solution:**
```bash
# Verify ros2_control is running
ros2 control list_controllers

# Check controller is active
ros2 control list_controllers | grep joint_trajectory_controller

# Verify controller_manager_name in moveit config
# Default: controller_manager
```

---

## Performance Tuning

### Planning Performance

In `ompl_planning.yaml`:
```yaml
planning_time: 5.0  # Increase for complex scenes
max_solution_segment_length: 0.01  # Decrease for smoother paths
simplify_solutions: true  # Enable path simplification
```

### Collision Checking

```yaml
collision_detection:
  plugin: collision_detection/BulletCollisionDetection  # Or FCL
  contacts_per_pair: 1  # Reduce for performance
```

### State Validity Checking

```yaml
state_validity_check_resolution: 0.01  # Increase for faster, less safe planning
```

---

## Optional: Database Support

### Install MongoDB for trajectory storage

```bash
# Install MongoDB
sudo apt install mongodb

# Install warehouse_ros
sudo apt install ros-jazzy-warehouse-ros-mongo

# Or build from source
cd ~/moveit2_ws/src
git clone https://github.com/moveit/warehouse_ros.git -b jazzy
git clone https://github.com/moveit/warehouse_ros_mongo.git -b jazzy
cd ~/moveit2_ws
colcon build --packages-select warehouse_ros warehouse_ros_mongo
```

**Usage:**
- Store and retrieve robot states
- Save successful trajectories
- Query planning scenes

---

## Uninstalling

```bash
# Remove workspace
rm -rf ~/moveit2_ws

# Remove from bashrc
sed -i '/moveit2_ws/d' ~/.bashrc
```

---

## Next Steps

After installing MoveIt2:

1. **Install additional tools** - See [ADDITIONAL_TOOLS_INSTALL.md](ADDITIONAL_TOOLS_INSTALL.md)
2. **Generate MoveIt config** - See [CLAUDE.md](CLAUDE.md#moveit2-regeneration)
3. **Test with humanoid arm** - See [README.md](README.md#5-moveit2-motion-planning)
4. **Configure servo for teleoperation** - See `humanoid_arm_teleop` package

---

## Key Files for This Project

- **[humanoid_arm_moveit_config/](humanoid_arm_moveit_config/)** - MoveIt configuration for 5-DOF arm
- **[demo.launch.py](humanoid_arm_moveit_config/launch/demo.launch.py)** - Standalone MoveIt demo
- **[full_system.launch.py](humanoid_arm_bringup/launch/full_system.launch.py)** - Gazebo + MoveIt + RViz

---

## References

- MoveIt2 Documentation: https://moveit.picknik.ai/jazzy/index.html
- Tutorials: https://moveit.picknik.ai/jazzy/doc/tutorials/tutorials.html
- API Reference: https://moveit.picknik.ai/jazzy/api/html/
- Setup Assistant: https://moveit.picknik.ai/jazzy/doc/examples/setup_assistant/setup_assistant_tutorial.html
- MoveIt Servo: https://moveit.picknik.ai/jazzy/doc/examples/realtime_servo/realtime_servo_tutorial.html

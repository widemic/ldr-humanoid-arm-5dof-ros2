# ros2_control Installation from Source

Complete guide for building and installing ros2_control and ros2_controllers from source for ROS2 Jazzy.

## Prerequisites

- **ROS2 Jazzy installed** (from source or binary) - See [ROS2_INSTALL.md](ROS2_INSTALL.md)
- **Ubuntu 24.04** (Noble Numbat)
- **Build tools** (colcon, git, cmake)

---

## Overview

The ros2_control framework consists of several repositories:

1. **ros2_control** - Core hardware interface and controller manager
2. **ros2_controllers** - Standard controllers (trajectory, position, velocity, effort)
3. **control_msgs** - Message definitions for controllers
4. **realtime_tools** - Real-time safe utilities
5. **control_toolbox** - PID controllers and filters
6. **ros2_control_test_assets** - Mock hardware for testing

---

## 1. Create Workspace

```bash
# Create separate workspace for ros2_control
mkdir -p ~/ros2_control_ws/src
cd ~/ros2_control_ws/src
```

**Note:** You can also add these to your existing ROS2 workspace if preferred.

---

## 2. Clone Source Repositories

### Method 1: Using vcstool (Recommended)

Create a `.repos` file:

```bash
cd ~/ros2_control_ws
cat > ros2_control.repos << 'EOF'
repositories:
  ros2_control:
    type: git
    url: https://github.com/ros-controls/ros2_control.git
    version: jazzy
  ros2_controllers:
    type: git
    url: https://github.com/ros-controls/ros2_controllers.git
    version: jazzy
  control_msgs:
    type: git
    url: https://github.com/ros-controls/control_msgs.git
    version: jazzy
  realtime_tools:
    type: git
    url: https://github.com/ros-controls/realtime_tools.git
    version: jazzy
  control_toolbox:
    type: git
    url: https://github.com/ros-controls/control_toolbox.git
    version: jazzy
  ros2_control_test_assets:
    type: git
    url: https://github.com/ros-controls/ros2_control.git
    version: jazzy
EOF

# Import repositories
vcs import src < ros2_control.repos
```

### Method 2: Manual clone

```bash
cd ~/ros2_control_ws/src

# Core ros2_control framework
git clone https://github.com/ros-controls/ros2_control.git -b jazzy

# Standard controllers
git clone https://github.com/ros-controls/ros2_controllers.git -b jazzy

# Control messages
git clone https://github.com/ros-controls/control_msgs.git -b jazzy

# Realtime tools
git clone https://github.com/ros-controls/realtime_tools.git -b jazzy

# PID and filters
git clone https://github.com/ros-controls/control_toolbox.git -b jazzy
```

---

## 3. Install Dependencies

```bash
cd ~/ros2_control_ws

# Source ROS2 (adjust path if installed from source)
source /opt/ros/jazzy/setup.bash
# OR if built from source:
# source ~/ros2_jazzy/install/setup.bash

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -y -r
```

**Key dependencies installed:**
- `rclcpp`, `rclpy` (ROS2 client libraries)
- `pluginlib` (plugin management)
- `hardware_interface` (hardware abstraction)
- `controller_interface` (controller base classes)
- `generate_parameter_library` (parameter handling)
- `tinyxml2` (XML parsing)
- `yaml-cpp` (YAML parsing)

---

## 4. Build ros2_control

```bash
cd ~/ros2_control_ws

# Build in release mode
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Or with parallel build (faster)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
```

**Build time:** 5-15 minutes depending on CPU

### Build specific packages only

```bash
# Build only ros2_control core
colcon build --packages-select controller_manager hardware_interface controller_interface

# Build only controllers
colcon build --packages-select \
  joint_trajectory_controller \
  position_controllers \
  velocity_controllers \
  effort_controllers \
  joint_state_broadcaster
```

---

## 5. Source the Workspace

```bash
cd ~/ros2_control_ws
source install/setup.bash
```

### Add to ~/.bashrc

```bash
# Add after ROS2 sourcing
echo "source ~/ros2_control_ws/install/setup.bash" >> ~/.bashrc
```

**Important:** Source order matters! ROS2 must be sourced before ros2_control workspace.

---

## 6. Verify Installation

### Check controller_manager
```bash
ros2 pkg list | grep controller_manager
# Should output: controller_manager
```

### Check available controllers
```bash
ros2 pkg list | grep controller
```

**Expected output:**
```
admittance_controller
ackermann_steering_controller
bicycle_steering_controller
controller_interface
controller_manager
controller_manager_msgs
diff_drive_controller
effort_controllers
force_torque_sensor_broadcaster
forward_command_controller
gripper_controllers
imu_sensor_broadcaster
joint_state_broadcaster
joint_trajectory_controller
pid_controller
position_controllers
range_sensor_broadcaster
steering_controllers_library
tricycle_controller
tricycle_steering_controller
velocity_controllers
```

### Test controller_manager
```bash
ros2 run controller_manager --help
# Should show controller_manager usage
```

---

## 7. Package Details

### ros2_control Core Packages

#### controller_manager
- Loads, configures, and manages controllers
- Handles controller lifecycle (inactive, active, etc.)
- Switches between controllers
- Main node: `ros2_control_node`

#### hardware_interface
- Defines hardware abstraction layer
- Command/state interfaces for joints
- System, actuator, and sensor components
- Base classes for custom hardware

#### controller_interface
- Base classes for controllers
- Real-time safe controller loop
- Parameter management
- Chainable controllers support

#### transmission_interface
- Mechanical transmissions (simple, differential)
- Maps actuator space to joint space

---

### ros2_controllers Packages

#### joint_trajectory_controller
**Most important for this project!**
- Executes joint trajectories with position/velocity/effort control
- PID gains for effort control
- Trajectory tolerances and constraints
- Used by MoveIt2

**Command interface:** `position`, `velocity`, or `effort`
**State interfaces:** `position`, `velocity`, `effort`

#### joint_state_broadcaster
- Publishes `/joint_states` topic
- Required for robot_state_publisher and RViz

#### position_controllers
- `JointGroupPositionController` - Group position control
- `forward_command_controller` - Direct position forwarding

#### velocity_controllers
- `JointGroupVelocityController` - Group velocity control

#### effort_controllers
- `JointGroupEffortController` - Group effort/torque control

#### force_torque_sensor_broadcaster
- Publishes force/torque sensor data
- Used for contact sensing (important for this project)

#### imu_sensor_broadcaster
- Publishes IMU data (orientation, angular velocity, linear acceleration)

---

## 8. Testing Installation

### Test with demo robot

```bash
# Terminal 1: Launch demo robot with ros2_control
ros2 launch ros2_control_test_nodes test_forward_position_controller.launch.py

# Terminal 2: Check controllers
ros2 control list_controllers

# Terminal 3: Send command
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, 0.5, 0.5]"
```

### Check hardware interfaces

```bash
ros2 control list_hardware_interfaces
```

---

## 9. Build Options

### Debug Build (for development)
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### With Tests
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test
colcon test-result --verbose
```

### Clean Rebuild
```bash
cd ~/ros2_control_ws
rm -rf build install log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## 10. Gazebo Integration

For Gazebo Harmonic integration (required for this project):

```bash
# Install Gazebo ros2_control plugin (binary)
sudo apt install ros-jazzy-gz-ros2-control

# Or build from source
cd ~/ros2_control_ws/src
git clone https://github.com/ros-controls/gz_ros2_control.git -b jazzy
cd ~/ros2_control_ws
rosdep install --from-paths src --ignore-src -y -r
colcon build --symlink-install --packages-select gz_ros2_control
```

**Provides:**
- `gz_ros2_control/GazeboSimSystem` - Gazebo hardware interface
- Effort-based control in simulation
- Contact force feedback

---

## 11. Additional Controller Packages (Optional)

### Admittance Controller
```bash
cd ~/ros2_control_ws/src
git clone https://github.com/ros-controls/ros2_controllers.git -b jazzy
# Already included if you cloned ros2_controllers
```

### Custom Controllers
You can create your own controllers by:
1. Inheriting from `controller_interface::ControllerInterface`
2. Implementing lifecycle callbacks
3. Registering as pluginlib plugin

See [humanoid_arm_control](humanoid_arm_control/) for examples.

---

## 12. Updating ros2_control

To update to latest Jazzy version:

```bash
cd ~/ros2_control_ws

# Pull latest changes
vcs pull src

# Or manually:
cd src/ros2_control && git pull origin jazzy
cd ../ros2_controllers && git pull origin jazzy
cd ../..

# Reinstall dependencies
rosdep install --from-paths src --ignore-src -y -r

# Rebuild
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## Troubleshooting

### Issue: "Could not find controller_interface"

**Solution:**
```bash
# Ensure ros2_control is built and sourced
cd ~/ros2_control_ws
colcon build --packages-select controller_interface
source install/setup.bash
```

### Issue: Controller plugin not found

**Solution:**
```bash
# Check plugin is registered
ros2 pkg list | grep <controller_package>

# Verify plugin XML
cat install/<controller_package>/share/<controller_package>/plugin_description.xml

# Re-source workspace
source ~/ros2_control_ws/install/setup.bash
```

### Issue: Hardware interface not found

**Solution:**
```bash
# Check hardware component is built
colcon build --packages-select hardware_interface

# Verify URDF has correct ros2_control tags
# Check <plugin> tag matches your hardware interface class
```

### Issue: Controller fails to load with "resource not found"

**Solution:**
```bash
# Rebuild specific controller
colcon build --packages-select joint_trajectory_controller --cmake-clean-cache

# Check controller parameters match expected
ros2 param list /controller_manager
```

### Issue: Real-time performance issues

**Solution:**
```bash
# Install real-time kernel (see ADDITIONAL_TOOLS_INSTALL.md)

# Set thread priority (requires RT kernel)
sudo setcap cap_sys_nice+ep install/controller_manager/lib/controller_manager/ros2_control_node

# Check for real-time violations
dmesg | grep -i "real-time"
```

---

## Performance Tuning

### Controller Manager Update Rate

In your `controllers.yaml`:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz (adjust based on hardware)
```

**Guidelines:**
- Mock hardware: 50-100 Hz
- Gazebo simulation: 100-200 Hz
- Real hardware: 100-1000 Hz (depends on motors)

### Use Real-time Executor

For hardware control:
```cpp
rclcpp::executors::RealTimeExecutor executor;
```

---

## Uninstalling

```bash
# Remove workspace
rm -rf ~/ros2_control_ws

# Remove from bashrc
sed -i '/ros2_control_ws/d' ~/.bashrc
```

---

## Next Steps

After installing ros2_control:

1. **Install MoveIt2 from source** - See [MOVEIT2_INSTALL.md](MOVEIT2_INSTALL.md)
2. **Install Gazebo and tools** - See [ADDITIONAL_TOOLS_INSTALL.md](ADDITIONAL_TOOLS_INSTALL.md)
3. **Build humanoid arm workspace** - See [README.md](README.md)
4. **Configure controllers** - See [CLAUDE.md](CLAUDE.md) and [PID_README.md](PID_README.md)

---

## Key Files for This Project

- **[controllers.yaml](humanoid_arm_control/config/controllers.yaml)** - Controller configuration with PID gains
- **[humanoid_arm_5dof.ros2_control.xacro](humanoid_arm_description/urdf/humanoid_arm_5dof.ros2_control.xacro)** - Hardware interface definition
- **[PID_README.md](PID_README.md)** - PID tuning guide for joint_trajectory_controller

---

## References

- ros2_control Documentation: https://control.ros.org/jazzy/index.html
- ros2_controllers: https://control.ros.org/jazzy/doc/ros2_controllers/doc/controllers_index.html
- Hardware Interface: https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html
- Controller Chaining: https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/controller_chaining.html

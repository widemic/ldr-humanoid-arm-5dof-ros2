# Software Guide: ROS2 Tools and GUIs

This guide covers the various software tools, GUIs, and utilities for working with the humanoid arm in ROS2.

---

## Table of Contents
1. [Control & Visualization Tools](#control--visualization-tools)
2. [PID Tuning Tools](#pid-tuning-tools)
3. [Diagnostic & Monitoring Tools](#diagnostic--monitoring-tools)
4. [Development Tools](#development-tools)

---

## Control & Visualization Tools

### rqt_joint_trajectory_controller

**Purpose:** GUI for sending position commands to the joint trajectory controller. Allows manual control of all joints with sliders.

**Launch:**
```bash
# Start robot first
ros2 launch humanoid_arm_bringup full_system.launch.py

# Launch the GUI (in another terminal)
rqt_joint_trajectory_controller
```

**Features:**
- Individual sliders for each joint
- Send coordinated multi-joint commands
- Real-time position control
- Works with `joint_trajectory_controller`

**Topics:**
- Publishes to: `/joint_trajectory_controller/joint_trajectory`
- Subscribes to: `/joint_states` (for feedback)

**Use Cases:**
- Manual testing of joint ranges
- Quick motion validation
- Interactive robot control
- Testing before running automated scripts

**Limitations:**
- All joints move together (coordinated control)
- Uses trajectory controller, which tries to stabilize all joints
- Not ideal for isolated single-joint PID tuning (use `pid_tuning.launch.py` instead)

---

### rqt_publisher

**Purpose:** General-purpose ROS2 topic publisher GUI. Can publish any message type to any topic.

**Launch:**
```bash
rqt_publisher
```

**Features:**
- Drop-down topic and message type selection
- Manual field editing for message contents
- Publish once or at a fixed rate
- Supports all standard ROS2 message types

**Topics:**
- User-configurable (any topic)

**Use Cases:**
- Testing single joint controllers (e.g., `/base_rotation_joint_position_controller/command`)
- Publishing test commands during development
- Debugging topic communication
- Quick manual position commands

**Example for Single Joint Control:**
1. Start `pid_tuning.launch.py`
2. Launch `rqt_publisher`
3. Select topic: `/base_rotation_joint_position_controller/command`
4. Message type: `std_msgs/msg/Float64`
5. Set `data` field to desired position (e.g., `0.5`)
6. Click publish

---

### RViz2

**Purpose:** 3D visualization tool for ROS2. Displays robot model, TF transforms, sensor data, and more.

**Launch:**
```bash
# Standalone
rviz2

# With configuration file
rviz2 -d /path/to/config.rviz

# Included in full_system launch
ros2 launch humanoid_arm_bringup full_system.launch.py
```

**Features:**
- Robot model visualization (URDF)
- TF transform tree display
- Joint state visualization
- Sensor data (cameras, point clouds, etc.)
- Interactive markers (MoveIt)
- Trajectory visualization

**Topics:**
- `/robot_description` - Robot URDF
- `/joint_states` - Joint positions/velocities
- `/tf` and `/tf_static` - Transform tree

**Use Cases:**
- Visualizing robot state
- MoveIt motion planning interface
- Debugging TF transforms
- Monitoring sensor data
- Checking collision meshes

**Note:** Not needed for isolated PID tuning (removed from `pid_tuning.launch.py`)

---

### Gazebo Harmonic (gz-sim)

**Purpose:** Physics simulation environment. Simulates realistic dynamics, gravity, inertia, friction, contacts.

**Launch:**
```bash
# Physics-only mode (no MoveIt)
ros2 launch humanoid_arm_bringup gazebo.launch.py

# Full system (Gazebo + MoveIt + RViz)
ros2 launch humanoid_arm_bringup full_system.launch.py

# PID tuning mode (Gazebo + single joint control)
ros2 launch humanoid_arm_bringup pid_tuning.launch.py
```

**Features:**
- Real physics simulation (gravity, inertia, dynamics)
- Effort-based control (torque commands)
- Contact/collision simulation
- Force/torque sensor simulation
- Multiple world environments (SDF files)

**Use Cases:**
- PID tuning (realistic dynamics)
- Contact manipulation testing
- Pre-hardware validation
- Physics parameter tuning
- Trajectory testing with realistic behavior

**Note:** Uses `effort` command interface, so PID gains directly control torque output (realistic oscillations).

---

## PID Tuning Tools

### PID Tuner GUI

**Purpose:** Interactive GUI for real-time PID parameter adjustment using sliders.

**Launch:**
```bash
# Start robot first (Gazebo or real hardware)
ros2 launch humanoid_arm_bringup full_system.launch.py

# In another terminal
ros2 run humanoid_arm_control pid_tuner_gui.py
```

**Features:**
- Real-time parameter updates (no restart needed)
- Sliders for P, I, D, I Clamp, and Feedforward Velocity gains
- Select any of the 5 joints
- Apply/Reset buttons
- Visual feedback on parameter changes

**Use Cases:**
- Real-time PID tuning
- Quick parameter experiments
- Testing gain effects on stability

**See:** [PID_README.md](PID_README.md) for complete tuning guide

---

### CLI PID Tuner

**Purpose:** Command-line tool for setting PID gains programmatically.

**Launch:**
```bash
ros2 run humanoid_arm_control tune_pid.py --joint base_rotation_joint --p 600 --d 20 --i 2
```

**Features:**
- Script-friendly (for automation)
- Set all PID parameters in one command
- Immediate parameter updates

**Use Cases:**
- Automated tuning scripts
- Batch parameter testing
- CI/CD integration

---

### Test Trajectory Generator

**Purpose:** Generates test trajectories for systematic PID tuning (step, sine, square wave, chirp).

**Launch:**
```bash
# Single test run
python3 humanoid_arm_control/scripts/test_trajectory.py --joint 0 --type step --amplitude 0.5 --reset

# Continuous testing (for real-time tuning)
python3 humanoid_arm_control/scripts/continuous_test.py --joint 0 --type sine --amplitude 0.5
```

**Features:**
- Multiple trajectory types (step, sine, square, chirp)
- Adjustable amplitude and frequency
- Reset option (move to zero first)
- Continuous mode for real-time PID adjustment

**Use Cases:**
- Systematic PID tuning
- System identification
- Testing tracking performance
- Characterizing oscillations/overshoot

**See:** [PID_README.md](PID_README.md) for detailed usage

---

### Isolated PID Tuning Launch

**Purpose:** Launch configuration for tuning a single joint in isolation (no multi-joint coordination).

**Launch:**
```bash
# Tune base rotation joint (default)
ros2 launch humanoid_arm_bringup pid_tuning.launch.py

# Tune specific joint
ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint_to_tune:=shoulder_pitch_joint

# Available joints:
# - base_rotation_joint
# - shoulder_pitch_joint
# - elbow_pitch_joint
# - wrist_pitch_joint
# - wrist_roll_joint
```

**What it does:**
- Launches Gazebo with physics simulation
- Spawns ONLY one joint controller (no interference from other joints)
- Uses effort interface (realistic PID behavior)
- Auto-launches `rqt_joint_trajectory_controller` GUI with sliders

**Features:**
- No RViz (not needed for PID tuning)
- No MoveIt (no trajectory planning interference)
- Single joint control (other joints uncontrolled, no oscillations)
- Effort-based control (PID gains affect torque)
- GUI with slider for easy position control

**Control the joint:**
1. **Via rqt_joint_trajectory_controller** (auto-launches):
   - Select controller from dropdown (e.g., `elbow_pitch_joint_position_controller`)
   - Use slider to set desired position
   - GUI sends trajectory commands automatically
   - Only the selected joint moves!

2. **Via command line (alternative):**
   ```bash
   ros2 topic pub /elbow_pitch_joint_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5]"
   ```

**Use Cases:**
- Isolated PID tuning for single joints
- Avoiding multi-joint coupling during tuning
- Clear observation of PID effects (no interference)
- Testing oscillations/overshoot on one joint at a time

**Workflow:**
1. Launch: `ros2 launch humanoid_arm_bringup pid_tuning.launch.py joint_to_tune:=elbow_pitch_joint`
2. Wait for `rqt_joint_trajectory_controller` GUI to appear (~9 seconds)
3. Select controller from dropdown: `elbow_pitch_joint_position_controller`
4. Move slider to send position commands
5. Observe joint behavior in Gazebo (watch for oscillations/overshoot)
6. Open PID tuner in another terminal: `ros2 run humanoid_arm_control pid_tuner_gui.py`
7. Adjust PID gains while testing with slider
8. Iterate until stable tracking with minimal overshoot

---

## Diagnostic & Monitoring Tools

### ros2 control CLI

**Purpose:** Command-line tools for managing ros2_control controllers.

**Commands:**
```bash
# List all controllers and their states
ros2 control list_controllers

# List available hardware interfaces
ros2 control list_hardware_interfaces

# Switch controllers (start/stop)
ros2 control switch_controllers --start-controllers joint_trajectory_controller --stop-controllers arm_position_controller

# Load a new controller
ros2 run controller_manager spawner joint_trajectory_controller
```

**Use Cases:**
- Checking controller status
- Switching between controllers
- Debugging hardware interfaces
- Verifying controller configuration

---

### ros2 topic CLI

**Purpose:** Monitor and inspect ROS2 topics.

**Commands:**
```bash
# Monitor joint states
ros2 topic echo /joint_states

# Monitor controller tracking performance
ros2 topic echo /joint_trajectory_controller/controller_state

# Show tracking errors only
ros2 topic echo /joint_trajectory_controller/controller_state --field error.positions

# Monitor force/torque sensors
ros2 topic echo /humanoid_arm/tcp_wrench

# List all topics
ros2 topic list

# Show topic info (publishers/subscribers)
ros2 topic info /joint_states

# Check message rate
ros2 topic hz /joint_states
```

---

### ros2 param CLI

**Purpose:** Get and set ROS2 parameters at runtime.

**Commands:**
```bash
# List all parameters for a controller
ros2 param list /joint_trajectory_controller

# Get PID gains
ros2 param get /joint_trajectory_controller gains.base_rotation_joint.p

# Set PID gains at runtime
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.p 600.0
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.d 20.0
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.i 2.0
```

**Use Cases:**
- Runtime PID tuning (alternative to GUI)
- Debugging parameter values
- Quick parameter changes

---

### rqt_console

**Purpose:** GUI for viewing ROS2 logs with filtering and severity levels.

**Launch:**
```bash
rqt_console
```

**Features:**
- Filter logs by severity (Debug, Info, Warn, Error, Fatal)
- Filter by node name
- Search log messages
- Export logs

**Use Cases:**
- Debugging controller errors
- Monitoring system warnings
- Finding error messages in complex launches

---

### rqt_graph

**Purpose:** Visualize ROS2 node and topic connections.

**Launch:**
```bash
rqt_graph
```

**Features:**
- Node graph visualization
- Topic connections
- Service connections
- Interactive graph exploration

**Use Cases:**
- Understanding system architecture
- Debugging communication issues
- Verifying topic connections

---

### rqt_plot

**Purpose:** Real-time plotting of ROS2 topic data.

**Launch:**
```bash
rqt_plot
```

**Features:**
- Real-time line plots
- Multiple topic fields on same plot
- Adjustable time window
- Export plot data

**Use Cases:**
- Plotting joint positions vs. time
- Visualizing PID tracking errors
- Monitoring sensor data trends
- System identification

**Example:**
```bash
# Plot joint positions
rqt_plot /joint_states/position[0] /joint_states/position[1]

# Plot tracking errors
rqt_plot /joint_trajectory_controller/controller_state/error/positions[0]
```

---

## Development Tools

### joint_state_publisher_gui

**Purpose:** GUI with sliders for manually setting joint positions (visualization only, no control).

**Launch:**
```bash
# Standalone (no controllers)
ros2 launch humanoid_arm_description display.launch.py
```

**Features:**
- Sliders for each joint
- Publishes to `/joint_states`
- Visual feedback in RViz

**Use Cases:**
- Inspecting robot model (URDF/meshes)
- Checking joint limits
- Visualizing poses without running controllers

**Note:** Does NOT control ros2_control systems (only publishes joint states for visualization). Use `rqt_joint_trajectory_controller` or `rqt_publisher` for actual control.

---

### MoveIt Setup Assistant

**Purpose:** GUI tool for generating MoveIt configuration packages.

**Launch:**
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

**Use Cases:**
- Creating MoveIt config packages
- Defining planning groups
- Setting up IK solvers
- Configuring collision checking

**Note:** Only needed when regenerating MoveIt configuration (URDF changes, new joints, etc.)

---

### xacro

**Purpose:** XML macro language for URDF files. Allows parameterization and reusability.

**Commands:**
```bash
# Process xacro to URDF
xacro humanoid_arm_5dof_ros2_control.urdf.xacro > output.urdf

# Check URDF validity
check_urdf output.urdf
```

**Use Cases:**
- Editing robot description
- Creating modular URDF files
- Debugging URDF syntax

---

### TF Tools

**Purpose:** Tools for working with coordinate transforms.

**Commands:**
```bash
# View entire TF tree
ros2 run tf2_tools view_frames

# Echo transform between two frames
ros2 run tf2_ros tf2_echo world base_link

# Monitor TF tree
ros2 run tf2_tools tf2_monitor
```

**Use Cases:**
- Debugging transform issues
- Verifying kinematic chain
- Checking for broken TF links

---

## Quick Reference Table

| Tool | Purpose | Launch/Command |
|------|---------|----------------|
| `rqt_joint_trajectory_controller` | Multi-joint manual control | `rqt_joint_trajectory_controller` |
| `rqt_publisher` | Publish to any topic | `rqt_publisher` |
| `pid_tuning.launch.py` | Isolated single-joint PID tuning | `ros2 launch humanoid_arm_bringup pid_tuning.launch.py` |
| `pid_tuner_gui.py` | Real-time PID adjustment | `ros2 run humanoid_arm_control pid_tuner_gui.py` |
| `test_trajectory.py` | Test trajectory generator | `python3 humanoid_arm_control/scripts/test_trajectory.py` |
| `rqt_plot` | Real-time data plotting | `rqt_plot` |
| `rqt_graph` | Node/topic graph | `rqt_graph` |
| `rqt_console` | Log viewer | `rqt_console` |
| `ros2 control` | Controller management | `ros2 control list_controllers` |
| `ros2 topic` | Topic monitoring | `ros2 topic echo /joint_states` |
| `ros2 param` | Parameter get/set | `ros2 param set /controller gains.joint.p 100` |

---

## See Also

- [PID_README.md](PID_README.md) - Complete PID tuning guide
- [CLAUDE.md](CLAUDE.md) - Full project documentation and architecture

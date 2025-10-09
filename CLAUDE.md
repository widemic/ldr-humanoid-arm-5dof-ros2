# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 Jazzy workspace for a 5-DOF humanoid robotic arm designed for whole-arm manipulation and contact-based interaction. Originally exported from SolidWorks, now fully migrated to ROS2 with ros2_control, MoveIt2, and Gazebo Harmonic simulation.

**Key Features:**
- Three simulation modes: mock hardware, Gazebo physics, full system (Gazebo + MoveIt + RViz)
- Ready for Robstride actuator hardware integration (ROBSTRIDE 04/03/02 motors)
- No gripper/hand - designed for whole-arm contact manipulation
- Multiple control modes: position, velocity, trajectory
- Force/torque sensing for contact-based tasks
- PID tuning tools with real-time GUI and test trajectory generator
- **Single source of truth for actuator specs:** All motor limits auto-loaded from [actuator_specs.yaml](humanoid_arm_control/config/actuator_specs.yaml)

## Build and Development Commands

### Build
```bash
# From workspace root
colcon build --symlink-install
source install/setup.bash

# Build specific package
colcon build --packages-select humanoid_arm_description

# Build with verbose output for debugging
colcon build --event-handlers console_direct+
```

## Launch Modes

The system supports three launch modes for different use cases:

### 1. Mock Hardware (Development/Testing)
**Use when:** Developing controllers, testing trajectories without physics
```bash
ros2 launch humanoid_arm_bringup robot.launch.py
```

**Characteristics:**
- Uses `mock_components/GenericSystem` - perfect position tracking
- No physics simulation (no gravity, inertia, or dynamics)
- Fast and stable - good for debugging logic
- **PID gains have minimal effect** (no oscillations possible)
- Controllers and MoveIt work normally

**Best for:**
- Controller development
- MoveIt configuration testing
- Trajectory validation
- Quick iteration without simulation overhead

### 2. Gazebo Simulation (Physics Only)
**Use when:** Testing physics behavior, PID tuning, contact dynamics
```bash
# Empty world
ros2 launch humanoid_arm_bringup gazebo.launch.py

# With specific world
ros2 launch humanoid_arm_bringup gazebo.launch.py world_file:=contact_manipulation_arena.sdf
```

**Characteristics:**
- Uses Gazebo Harmonic (gz-sim) with `gz_ros2_control/GazeboSimSystem`
- Real physics: gravity, inertia, friction, damping
- **Effort command interface:** PID gains control torque, enabling realistic oscillations/overshoot
- **PID tuning shows actual dynamic behavior** (no perfect tracking like mock hardware)
- Contact forces and collisions simulated
- No MoveIt or RViz (controller-level testing)

**Best for:**
- PID gain tuning (see [PID_README.md](PID_README.md))
- Contact manipulation testing
- Physics parameter validation
- Low-level controller behavior analysis

### 3. Full System (Gazebo + MoveIt + RViz)
**Use when:** End-to-end testing with motion planning and visualization
```bash
ros2 launch humanoid_arm_bringup full_system.launch.py
```

**Characteristics:**
- Combines Gazebo physics + MoveIt motion planning + RViz visualization
- Complete system integration testing
- Interactive control via MoveIt RViz plugin
- **Real physics with effort control** - PID gains affect actual dynamics
- Launches in sequence: Gazebo → controllers → MoveIt → RViz
- Best for validating PID-tuned behavior with motion planning

**Best for:**
- Complete system validation
- Motion planning with realistic physics
- Demo and visualization
- Pre-hardware integration testing

### 4. Visualization Only (No Controllers)
**Use when:** Inspecting robot model, checking transforms
```bash
ros2 launch humanoid_arm_description display.launch.py
```

**Characteristics:**
- RViz with joint_state_publisher_gui
- Manual joint position sliders
- No controllers or physics
- Quick model inspection

## Additional Launch Commands

### Teleoperation

**Joystick control:**
```bash
# Start robot (any mode) first, then in another terminal:
ros2 launch humanoid_arm_teleop joystick_teleop.launch.py

# Or run node directly
ros2 run humanoid_arm_teleop joystick_teleop_node
```

**Keyboard control:**
```bash
ros2 run humanoid_arm_teleop keyboard_teleop_node
```

**Contact manipulation mode:**
```bash
# Enables force-based control using F/T sensors
ros2 run humanoid_arm_teleop contact_manipulation_node
```

### MoveIt2 Standalone (with mock hardware)
```bash
# Start robot.launch.py first, then:
ros2 launch humanoid_arm_moveit_config demo.launch.py
```

## Controller Management

### List and Inspect Controllers
```bash
# List active controllers
ros2 control list_controllers

# Check controller state
ros2 control list_hardware_interfaces

# View controller parameters
ros2 param list /joint_trajectory_controller
```

### Controller Switching
```bash
# Switch between trajectory and position control
ros2 control switch_controllers \
  --start-controllers joint_trajectory_controller \
  --stop-controllers arm_position_controller

# Load new controller
ros2 run controller_manager spawner joint_trajectory_controller
```

### PID Parameter Tuning
```bash
# Set PID gains at runtime (Gazebo or real hardware only)
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.p 600.0
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.d 20.0
ros2 param set /joint_trajectory_controller gains.base_rotation_joint.i 2.0

# Launch interactive PID tuner GUI
ros2 run humanoid_arm_control pid_tuner_gui.py

# Run test trajectories for systematic tuning
python3 humanoid_arm_control/scripts/test_trajectory.py --joint 0 --type step --amplitude 0.5 --reset
```

**See [PID_README.md](PID_README.md) for complete PID tuning guide.**

## Testing and Diagnostics

### Monitor Robot State
```bash
# Joint positions and velocities
ros2 topic echo /joint_states

# Controller tracking performance
ros2 topic echo /joint_trajectory_controller/controller_state

# Tracking errors only
ros2 topic echo /joint_trajectory_controller/controller_state --field error.positions

# Check TF tree
ros2 run tf2_tools view_frames
```

### Monitor Contact Forces
```bash
ros2 topic echo /humanoid_arm/tcp_wrench
ros2 topic echo /humanoid_arm/forearm_wrench
ros2 topic echo /humanoid_arm/wrist_wrench
```

### Test Joystick Input
```bash
ros2 run joy joy_node
ros2 topic echo /joy
```

### Diagnostics
```bash
# System diagnostics
ros2 topic echo /diagnostics

# Controller manager activity
ros2 topic echo /controller_manager/activity
```

## Architecture Overview

### Package Structure

**5 main packages in workspace root:**

1. **`humanoid_arm_description/`** - Robot model
   - URDF/Xacro files with ros2_control tags
   - STL meshes from SolidWorks
   - Gazebo plugins and sensor definitions
   - Display launch file (RViz with joint_state_publisher_gui)
   - Joint dynamics: damping=0.1, friction=0.1 (tuned for realistic simulation)

2. **`humanoid_arm_control/`** - Hardware interface and controllers
   - Mock hardware interface for development
   - Gazebo hardware interface (gz_ros2_control)
   - Controller configurations (trajectory, position, velocity)
   - PID tuning tools: `pid_tuner_gui.py`, `test_trajectory.py`
   - Force/torque sensor broadcasters

3. **`humanoid_arm_moveit_config/`** - MoveIt2 configuration
   - Generated via moveit_setup_assistant
   - Planning groups, IK solvers, collision checking
   - Motion planning pipelines (OMPL)
   - Custom MoveIt launch files for simulation

4. **`humanoid_arm_bringup/`** - System launch files
   - `robot.launch.py` - Mock hardware mode
   - `gazebo.launch.py` - Gazebo physics simulation
   - `full_system.launch.py` - Complete system (Gazebo + MoveIt + RViz)
   - World files (SDF format) in `worlds/` directory
   - System-level configuration

5. **`humanoid_arm_teleop/`** - Teleoperation nodes
   - Joystick control (PlayStation/Xbox controllers)
   - Keyboard teleop
   - Contact manipulation modes
   - Multiple control modes (joint, IK, predefined poses)

### Joint Configuration (5-DOF)

**Actuated joints with Robstride motors:**
1. `base_rotation_joint` - Base rotation (-3.14 to 3.14 rad) - **ROBSTRIDE 04** (120 Nm, 20.94 rad/s)
2. `shoulder_pitch_joint` - Shoulder pitch (-0.55 to 3.1 rad) - **ROBSTRIDE 04** (120 Nm, 20.94 rad/s)
3. `elbow_pitch_joint` - Elbow pitch (-3.14 to 3.14 rad) - **ROBSTRIDE 03** (60 Nm, 20.42 rad/s)
4. `wrist_pitch_joint` - Wrist pitch (-0.31 to 2.8 rad) - **ROBSTRIDE 03** (60 Nm, 20.42 rad/s)
5. `wrist_roll_joint` - Wrist roll (-3.14 to 3.14 rad) - **ROBSTRIDE 02** (17 Nm, 42.94 rad/s)

**Motor Specifications:**
All motor specs (torque, speed, electrical parameters) are defined in [actuator_specs.yaml](humanoid_arm_control/config/actuator_specs.yaml) and automatically loaded by URDF. See [ACTUATOR_LIMITS.md](ACTUATOR_LIMITS.md) for complete documentation.

**Joint Dynamics (URDF):**
- `damping`: 0.1 (realistic for brushless motors with gearboxes)
- `friction`: 0.1 (Coulomb friction)
- Original values (0.3-0.7) were reduced for better PID tuning visibility

**Note:** Original SolidWorks export had joints named joint2, joint3, joint6, joint9, joint11 plus fixed structural joints. These are now semantically renamed.

### ros2_control Architecture

**Hardware Interfaces:**
1. **Mock hardware** (`mock_components/GenericSystem`)
   - Perfect position tracking, no dynamics
   - Used in `robot.launch.py`
   - Parameters: `fake_hardware: true`, `slowdown: 3.0`
   - **Note:** PID gains have minimal effect (perfect tracking)

2. **Gazebo hardware** (`gz_ros2_control/GazeboSimSystem`)
   - Real physics simulation with **effort command interface**
   - Used in `gazebo.launch.py` and `full_system.launch.py`
   - Parameters: `use_fake_hardware: false`
   - **PID gains control torque output** → realistic dynamics with oscillations
   - Joint command interfaces: position, velocity, **effort** (all available)
   - Controller uses **effort interface** for realistic PID behavior

3. **Real hardware** (future)
   - CAN-based Robstride motor interface
   - To be implemented in `humanoid_arm_control/src/`
   - Will use same effort-based control for consistent behavior

**Available Controllers:**
- `joint_trajectory_controller` - Coordinated multi-joint motion (primary for MoveIt)
  - **Command interface: effort** (torque control for realistic dynamics)
  - State interfaces: position, velocity, effort
  - PID gains: tunable at runtime, **actually control torque output**
  - Trajectory constraints: path tolerance, goal tolerance, goal time
  - **Note:** Uses effort control so PID gains directly affect simulated torque and show realistic oscillations/overshoot
- `arm_position_controller` - Group position control (all joints)
- `arm_velocity_controller` - Group velocity control
- Individual joint position controllers (5 separate controllers)
- Force/torque sensor broadcasters (tcp, forearm, wrist)
- IMU sensor broadcaster

**Controller Switching:**
Trajectory and position controllers are mutually exclusive (claim same command interfaces). Switch between them using `ros2 control switch_controllers`.

### URDF/Xacro Structure

**Main file:** `humanoid_arm_5dof_ros2_control.urdf.xacro`
- Imports macro URDF: `humanoid_arm_5dof_macro.urdf.xacro`
- Imports ros2_control: `humanoid_arm_5dof.ros2_control.xacro`
- Imports Gazebo config: `humanoid_arm_5dof.gazebo.xacro`
- Parameters: use_fake_hardware, fake_sensor_commands, slowdown

**Key features:**
- Modular xacro macros for reusability
- Contact sensors on arm links (force/torque)
- Gazebo Harmonic plugins and sensor definitions
- Proper inertial properties for simulation
- Joint dynamics tuned for realistic behavior

### Configuration Files

**`humanoid_arm_control/config/controllers.yaml`**
- controller_manager update rate (100 Hz)
- All controller configurations with parameters
- **Command interface: effort** (line 62) - **Critical for realistic dynamics**
- **PID gains** for joint_trajectory_controller (control torque output):
  - `base_rotation_joint`: P=150, D=8, I=5 (default, tune as needed)
  - Other joints: P=80-180, D=5-10, I=4-8
  - `i_clamp`: 12-25 (prevents integral windup)
  - `ff_velocity_scale`: 0.06-0.12 (feedforward for smooth tracking)
- **Trajectory constraints**:
  - `goal_time`: 10.0 seconds (increased for effort control dynamics)
  - `trajectory` tolerance: 0.5-2.0 rad (relaxed for effort-based control)
  - `goal` tolerance: 0.1-0.5 rad (final position tolerance)
  - `stopped_velocity_tolerance`: 1.0 rad/s (relaxed for realistic dynamics)
- Sensor broadcaster frame IDs and topics

**Important:**
- Tolerances relaxed significantly for effort control (no perfect tracking like position interface)
- PID gains directly control torque → visible oscillations with high P or low D
- To see PID effects, must use Gazebo (effort control), not mock hardware

**`humanoid_arm_description/config/ros2_control.yaml`** (if exists)
- Hardware interface parameters
- Joint limits (safety limits tighter than URDF)
- Robstride motor specifications (motor types, CAN IDs, gear ratios)
- CAN interface config (bitrate, timeout, retry)

**`humanoid_arm_teleop/config/joystick_config.yaml`**
- Button/axis mappings for PlayStation/Xbox controllers
- Control mode settings
- Velocity scaling factors
- Safety parameters (deadman switch, e-stop)

### Simulation Environments

**Gazebo Harmonic (gz-sim):**
- Engine: Gazebo Harmonic (not Gazebo Classic)
- Integration: `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`
- Physics: Realistic inertia, gravity, friction, damping

**Available worlds (SDF format):**
1. `empty_world.sdf` - Basic testing, flat ground
2. `contact_manipulation_arena.sdf` - Objects for pushing/contact tasks
3. `whole_arm_workspace.sdf` - Workspace with varied surfaces for whole-arm manipulation
4. `assembly_tasks.sdf` - Assembly and precision manipulation scenarios
5. `obstacle_course.sdf` - Navigation and collision avoidance testing

**World location:** `humanoid_arm_bringup/worlds/`

**Usage:**
```bash
ros2 launch humanoid_arm_bringup gazebo.launch.py world_file:=contact_manipulation_arena.sdf
ros2 launch humanoid_arm_bringup full_system.launch.py world_file:=whole_arm_workspace.sdf
```

## PID Tuning Guide

**See [PID_README.md](PID_README.md) for complete guide.**

### Quick Reference

**Key insight:** PID tuning only works with **Gazebo (effort control) or real hardware**. Mock hardware has perfect tracking (no oscillations possible).

**Why effort control matters:**
- **Position interface** → Gazebo uses internal perfect position controller → your PID gains are ignored
- **Effort interface** → Your PID gains calculate torque → Gazebo applies torque → realistic dynamics with oscillations
- System uses **effort command interface** for this reason (see `controllers.yaml` line 62)

**Current values (base_rotation_joint, needs tuning with effort control):**
- P = 150.0 (default starting point for effort control)
- D = 8.0 (damping)
- I = 5.0 (prevents steady-state error)
- I Clamp = 20.0
- FF Velocity = 0.1

**Note:** These values are for **effort control** (torque-based). With position control, much higher gains (P=600) were needed. Effort control requires retuning from scratch.

**Tools:**
1. **PID Tuner GUI** - Interactive sliders for real-time tuning
   ```bash
   ros2 run humanoid_arm_control pid_tuner_gui.py
   ```

2. **CLI PID Tuner** - Command-line PID adjustment
   ```bash
   ros2 run humanoid_arm_control tune_pid.py --joint base_rotation_joint --p 600 --d 20 --i 2
   ```

3. **Test Trajectory Generator** - Single-run systematic testing (step, square, sine, chirp)
   ```bash
   python3 humanoid_arm_control/scripts/test_trajectory.py --joint 0 --type step --amplitude 0.5 --reset
   ```

4. **Continuous Test Generator** - Runs indefinitely for real-time tuning
   ```bash
   python3 humanoid_arm_control/scripts/continuous_test.py --joint 0 --type sine --amplitude 0.5
   # Adjust PID with GUI or CLI while this runs continuously
   ```

**Tuning workflow (effort control):**
1. Launch Gazebo: `ros2 launch humanoid_arm_bringup full_system.launch.py`
2. Start with P=100, I=0, D=0
3. Increase P until you see oscillations or overshoot
4. Add D to dampen oscillations (start with D = P/10)
5. Add small I only if steady-state error persists (I = P/50)
6. Test with `test_trajectory.py --reset` or MoveIt planning
7. **Expect to see oscillations** - this means PID is working correctly!

**With effort control, you WILL see:**
- Oscillations with high P and low D
- Overshoot with insufficient D
- Slower tracking (not perfect like position interface)
- Velocity errors during motion

## Hardware Integration

### Transitioning to Real Hardware

The project is designed for easy hardware transition:

1. **Change launch parameter:**
   ```bash
   ros2 launch humanoid_arm_bringup robot.launch.py use_fake_hardware:=false
   ```

2. **CAN interface setup:**
   ```bash
   sudo ip link set can0 up type can bitrate 1000000
   ```

3. **Motor configuration:**
   - CAN IDs: 0x01 to 0x05
   - Motor types: RMD-X8-PRO (base, shoulder), RMD-X6 (elbow, wrist pitch), RMD-X4 (wrist roll)
   - Encoder resolution: 16384 counts/rev
   - Torque/speed limits documented in hardware specs

4. **Hardware interface implementation:**
   - Implement `hardware_interface::SystemInterface` in `humanoid_arm_control/src/`
   - CAN communication using Robstride SDK or custom implementation
   - Handle position, velocity, and effort commands

## Development Notes

### When Editing URDF/Xacro Files
- Always edit the `.xacro` files, never the generated `.urdf`
- Test changes: `xacro humanoid_arm_5dof_ros2_control.urdf.xacro > test.urdf`
- Validate: `check_urdf test.urdf`
- Visualize: `ros2 launch humanoid_arm_description display.launch.py`
- **Rebuild after changes:** `colcon build --packages-select humanoid_arm_description --symlink-install`

### Adding New Controllers
1. Add controller to `controllers.yaml` under `controller_manager`
2. Configure controller parameters in same file
3. Update launch files to spawn the controller
4. Ensure no command interface conflicts with existing controllers
5. Test in mock hardware first, then Gazebo

### Creating New Gazebo Worlds
- Use SDF format (`.sdf` files)
- Place in `humanoid_arm_bringup/worlds/`
- Reference objects and plugins for contact sensing
- Test physics parameters (friction, contact coefficients)
- Launch with: `ros2 launch humanoid_arm_bringup gazebo.launch.py world_file:=your_world.sdf`

### MoveIt2 Regeneration
If URDF changes significantly, regenerate MoveIt config:
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
# Load humanoid_arm_5dof_ros2_control.urdf.xacro
# Regenerate into humanoid_arm_moveit_config/
```

### Adjusting Joint Damping/Friction
Joint dynamics are defined in `humanoid_arm_5dof_macro.urdf.xacro`:
```xml
<dynamics damping="0.1" friction="0.1"/>
```

**Current values (0.1):** Realistic for brushless motors, allows PID oscillations to be visible

**If you want:**
- More damping (stable, harder to see PID effects): 0.5-1.0
- Less damping (aggressive, clear oscillations): 0.0-0.05
- Original values: 0.3-0.7 (good for stability, masks some PID behavior)

**After changing, rebuild:** `colcon build --packages-select humanoid_arm_description`

## Common Issues

### Mock Hardware vs. Gazebo Confusion

**Problem:** "PID tuning doesn't show oscillations"

**Root causes:**
1. **Mock hardware** (`robot.launch.py`) has perfect tracking → no PID effects visible
2. **Position command interface** → Gazebo uses internal perfect controller → your PID ignored

**Solution:** Use Gazebo with **effort command interface** (already configured):
```bash
ros2 launch humanoid_arm_bringup full_system.launch.py
```

The system now uses **effort interface** in `controllers.yaml` (line 62), so PID gains control torque and show realistic oscillations.

**To verify effort control is active:**
```bash
ros2 control list_hardware_interfaces
# Should show: base_rotation_joint/effort [available] [claimed]
```

### PATH_TOLERANCE_VIOLATED Error (-4)

**Problem:** Trajectory aborts with error code -4 during motion

**Cause:** Robot can't keep up with desired trajectory within tolerance limits. **Common with effort control** because tracking isn't perfect like position interface.

**Solutions (in order of preference):**
1. **Increase trajectory tolerance** in `controllers.yaml`:
   ```yaml
   constraints:
     base_rotation_joint:
       trajectory: 2.0  # Increased from 0.5 for effort control
   ```

2. **Increase PID gains** (P and D) for faster/better tracking:
   ```bash
   ros2 run humanoid_arm_control tune_pid.py --joint base_rotation_joint --p 300 --d 20
   ```

3. **Increase goal_time** to allow more time:
   ```yaml
   goal_time: 10.0  # Increased from 3.0 for effort control
   ```

4. **Slow down trajectories** in MoveIt or use longer durations

5. **Check joint isn't at velocity limit**

**Note:** Effort control naturally has larger tracking errors than position interface. Looser tolerances are expected and realistic.

### Controllers Not Loading
- Check `ros2 control list_controllers` for errors
- Verify YAML syntax in controllers.yaml
- Ensure controller plugin is in ros2_control XML
- Check hardware interface is active: `ros2 control list_hardware_interfaces`

### Gazebo Physics Unstable
- Adjust `slowdown` parameter in URDF launch args
- Check collision meshes (use simplified geometry)
- Verify joint damping/friction values (0.1 is good starting point)
- Reduce PID gains if oscillations are too aggressive

### Joint Limits Exceeded
- Software limits in `controllers.yaml` constraints
- URDF limits in joint definitions
- Check for singularities near joint limits
- Verify trajectory planning respects limits

### Teleoperation Not Responding
- Verify joystick permissions: `sudo chmod 666 /dev/input/js0`
- Check deadman switch is held (L1 on PlayStation)
- Monitor `/joy` topic for input data
- Ensure robot is launched and controllers are active

### Test Trajectory Script Fails with Error -4

**Problem:** `test_trajectory.py` returns result -4 immediately

**Cause:** Robot starting position is far from trajectory start (tolerance violated)

**Solution:** Use `--reset` flag to move to zero first:
```bash
python3 humanoid_arm_control/scripts/test_trajectory.py --joint 0 --type step --amplitude 0.5 --reset
```

### Gazebo Launch Fails - "gazebo_ros not found"

**Problem:** Old launch file looking for Gazebo Classic

**Solution:** Already fixed - uses Gazebo Harmonic now. Ensure packages installed:
```bash
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-gz-ros2-control
```

## Important Files

- **[ACTUATOR_LIMITS.md](ACTUATOR_LIMITS.md)** - Complete actuator specification documentation and configuration guide
- **[actuator_specs.yaml](humanoid_arm_control/config/actuator_specs.yaml)** - Single source of truth for all motor specifications
- **[PID_README.md](PID_README.md)** - Complete PID tuning guide with methods and tools
- **[controllers.yaml](humanoid_arm_control/config/controllers.yaml)** - Controller and PID configuration
- **[humanoid_arm_5dof_macro.urdf.xacro](humanoid_arm_description/urdf/humanoid_arm_5dof_macro.urdf.xacro)** - Joint definitions and dynamics (auto-loads actuator_specs.yaml)
- **[humanoid_arm_5dof.ros2_control.xacro](humanoid_arm_description/urdf/humanoid_arm_5dof.ros2_control.xacro)** - Hardware interface configuration
  - **Effort command interfaces** added to all 5 joints with motor torque limits

## Tips for Pre-Hardware Development

You have **2 weeks before Robstride motors arrive**. Recommended workflow:

1. **Use Gazebo for realistic testing:**
   ```bash
   ros2 launch humanoid_arm_bringup full_system.launch.py
   ```

2. **Tune PID gains systematically:**
   - Use `pid_tuner_gui.py` for interactive tuning
   - Use `test_trajectory.py --reset` for repeatable tests
   - Document good values in `controllers.yaml`

3. **Test contact manipulation:**
   - Use `contact_manipulation_arena.sdf` world
   - Monitor force/torque sensors
   - Develop contact-based control strategies

4. **Validate trajectories:**
   - Test with MoveIt motion planning
   - Ensure large movements don't violate tolerances
   - Verify joint limits are respected

5. **Prepare hardware interface:**
   - Study Robstride CAN protocol
   - Design hardware interface plugin structure
   - Test with CAN simulation if possible

When hardware arrives, the tuned PID values and validated trajectories will transfer directly!
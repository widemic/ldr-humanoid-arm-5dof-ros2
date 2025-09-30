# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 Jazzy workspace for a 5-DOF humanoid robotic arm designed for whole-arm manipulation and contact-based interaction. Originally exported from SolidWorks, now fully migrated to ROS2 with ros2_control, MoveIt2, and Gazebo simulation.

**Key Features:**
- Simulation-first development with mock hardware
- Ready for Robstride actuator hardware integration
- No gripper/hand - designed for whole-arm contact manipulation
- Multiple control modes: position, velocity, trajectory
- Force/torque sensing for contact-based tasks

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

### Launch Commands

**Basic visualization (no controllers):**
```bash
ros2 launch humanoid_arm_description display.launch.py
```

**Robot with fake hardware and controllers:**
```bash
ros2 launch humanoid_arm_bringup robot.launch.py
```

**Gazebo simulation:**
```bash
# Empty world
ros2 launch humanoid_arm_bringup gazebo.launch.py world_file:=empty_world.sdf

# Contact manipulation arena
ros2 launch humanoid_arm_bringup gazebo.launch.py world_file:=contact_manipulation_arena.sdf
```

**Teleoperation:**
```bash
# Start robot first, then in another terminal:
ros2 launch humanoid_arm_teleop joystick_teleop.launch.py
```

**MoveIt2 motion planning:**
```bash
# Start robot first, then:
ros2 launch humanoid_arm_moveit_config demo.launch.py
```

### Controller Management
```bash
# List active controllers
ros2 control list_controllers

# Switch controllers
ros2 control switch_controllers \
  --start-controllers joint_trajectory_controller \
  --stop-controllers arm_position_controller

# Load controller
ros2 run controller_manager spawner joint_trajectory_controller
```

### Testing and Diagnostics
```bash
# Monitor joint states
ros2 topic echo /joint_states

# Check TF tree
ros2 run tf2_tools view_frames

# Monitor contact forces
ros2 topic echo /humanoid_arm/tcp_wrench
ros2 topic echo /humanoid_arm/forearm_wrench

# Test joystick input
ros2 run joy joy_node
ros2 topic echo /joy
```

## Architecture Overview

### Package Structure

**5 main packages in workspace root:**

1. **`humanoid_arm_description/`** - Robot model
   - URDF/Xacro files with ros2_control tags
   - STL meshes from SolidWorks
   - Gazebo plugins and sensor definitions
   - Display launch file (RViz with joint_state_publisher_gui)

2. **`humanoid_arm_control/`** - Hardware interface and controllers
   - Mock hardware interface (simulation mode)
   - Hardware abstraction for future Robstride motors
   - Controller configurations (trajectory, position, velocity)
   - Force/torque sensor broadcasters

3. **`humanoid_arm_moveit_config/`** - MoveIt2 configuration
   - Generated via moveit_setup_assistant
   - Planning groups, IK solvers, collision checking
   - Motion planning pipelines (OMPL)

4. **`humanoid_arm_bringup/`** - System launch files
   - Main robot.launch.py (spawns robot + controllers)
   - Gazebo launch files
   - World files (SDF format)
   - System-level configuration

5. **`humanoid_arm_teleop/`** - Teleoperation nodes
   - Joystick control (PlayStation/Xbox controllers)
   - Keyboard teleop
   - Contact manipulation modes
   - Multiple control modes (joint, IK, predefined poses)

### Joint Configuration (5-DOF)

**Actuated joints with renamed semantics:**
1. `base_rotation_joint` - Base rotation (-3.0 to 3.0 rad) - RMD-X8-PRO
2. `shoulder_pitch_joint` - Shoulder pitch (-0.5 to 3.0 rad) - RMD-X8-PRO
3. `elbow_pitch_joint` - Elbow pitch (-3.0 to 3.0 rad) - RMD-X6
4. `wrist_pitch_joint` - Wrist pitch (-0.25 to 2.7 rad) - RMD-X6
5. `wrist_roll_joint` - Wrist roll (-3.0 to 3.0 rad) - RMD-X4

**Note:** Original SolidWorks export had joints named joint2, joint3, joint6, joint9, joint11 plus fixed structural joints. These are now semantically renamed.

### ros2_control Architecture

**Hardware Interface:**
- Uses `mock_components` plugin for simulation (fake_hardware: true)
- Configured for future CAN-based Robstride motor communication
- Implements command/state interfaces: position, velocity, effort

**Available Controllers:**
- `joint_trajectory_controller` - Coordinated multi-joint motion (primary for MoveIt)
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
- Gazebo physics and sensor plugins
- Proper inertial properties for simulation

### Configuration Files

**`humanoid_arm_description/config/ros2_control.yaml`**
- Hardware interface parameters (use_fake_hardware, slowdown)
- Joint limits (safety limits tighter than URDF)
- Robstride motor specifications (motor types, CAN IDs, gear ratios)
- CAN interface config (bitrate, timeout, retry)
- Contact sensing thresholds

**`humanoid_arm_control/config/controllers.yaml`**
- controller_manager update rate (100 Hz)
- All controller configurations with parameters
- Trajectory constraints and tolerances
- Sensor broadcaster frame IDs and topics

**`humanoid_arm_teleop/config/joystick_config.yaml`**
- Button/axis mappings for PlayStation/Xbox controllers
- Control mode settings
- Velocity scaling factors
- Safety parameters (deadman switch, e-stop)

### Simulation Environments

**Current Gazebo worlds (SDF format):**
1. `empty_world.sdf` - Basic testing, flat ground
2. `contact_manipulation_arena.sdf` - Objects for pushing/contact tasks

**Note:** Uses Gazebo Classic for now. Gazebo Harmonic/Fortress migration may be needed for full Jazzy compatibility.

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

3. **Motor configuration in `ros2_control.yaml`:**
   - CAN IDs: 0x01 to 0x05
   - Motor types: RMD-X8-PRO (base, shoulder), RMD-X6 (elbow, wrist pitch), RMD-X4 (wrist roll)
   - Encoder resolution: 16384 counts/rev
   - Torque/speed limits documented

**Hardware interface implementation:**
The `humanoid_arm_control/src/` directory should contain the real hardware interface plugin (currently uses mock). Implement `hardware_interface::SystemInterface` for Robstride CAN communication.

## Development Notes

### When Editing URDF/Xacro Files
- Always edit the `.xacro` files, never the generated `.urdf`
- Test changes: `xacro humanoid_arm_5dof_ros2_control.urdf.xacro > test.urdf`
- Validate: `check_urdf test.urdf`
- Visualize: `ros2 launch humanoid_arm_description display.launch.py`

### Adding New Controllers
1. Add controller to `controllers.yaml` under `controller_manager`
2. Configure controller parameters in same file
3. Update launch files to spawn the controller
4. Ensure no command interface conflicts with existing controllers

### Creating New Gazebo Worlds
- Use SDF format (`.sdf` files)
- Place in `humanoid_arm_bringup/worlds/`
- Reference objects and plugins for contact sensing
- Test physics parameters (friction, contact coefficients)

### MoveIt2 Regeneration
If URDF changes significantly, regenerate MoveIt config:
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
# Load humanoid_arm_5dof_ros2_control.urdf.xacro
# Regenerate into humanoid_arm_moveit_config/
```

## Common Issues

**Controllers not loading:**
- Check `ros2 control list_controllers` for errors
- Verify YAML syntax in controllers.yaml
- Ensure controller plugin is in ros2_control XML

**Gazebo physics unstable:**
- Adjust `slowdown` parameter in URDF launch args
- Check collision meshes (use simplified geometry)
- Verify joint damping/friction values

**Joint limits exceeded:**
- Software limits in `ros2_control.yaml` are tighter than URDF
- Adjust for safety margin vs. workspace needs
- Check for singularities near joint limits

**Teleoperation not responding:**
- Verify joystick permissions: `sudo chmod 666 /dev/input/js0`
- Check deadman switch is held (L1 on PlayStation)
- Monitor `/joy` topic for input data
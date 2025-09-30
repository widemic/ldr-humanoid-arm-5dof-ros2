# Humanoid Arm 5-DOF ROS2 Package

A comprehensive ROS2 workspace for a 5-degree-of-freedom humanoid robotic arm designed for whole-arm manipulation and contact-based interaction. This project emphasizes simulation-first development with easy transition to real Robstride actuator hardware.

## 🤖 Robot Specifications

- **5 Degrees of Freedom**: Base rotation, shoulder pitch, elbow pitch, wrist pitch, wrist roll
- **No gripper/hand**: Designed for whole-arm manipulation and contact-based tasks
- **Actuators**: Robstride motors (RMD-X8-PRO, RMD-X6, RMD-X4 series)
- **Control Modes**: Position, velocity, and effort control
- **Sensors**: Contact force sensors, IMU, joint encoders

## 📦 Package Structure

```
humanoid_arm_5dof_ros2/
├── humanoid_arm_description/     # Robot URDF, meshes, and description
├── humanoid_arm_control/         # Hardware interface and controllers
├── humanoid_arm_moveit_config/   # MoveIt2 configuration (generate with setup wizard)
├── humanoid_arm_bringup/         # Launch files and system configuration
├── humanoid_arm_teleop/          # Teleoperation and joystick control
└── README.md                     # This file
```

## 🚀 Quick Start

### Prerequisites

- **ROS2 Jazzy** (recommended)
- **Gazebo Fortress/Harmonic** or **Gazebo Classic 11**
- **MoveIt2** for Jazzy
- **Joystick** (PlayStation/Xbox controller recommended)

### Installation

1. **Clone the repository:**
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> humanoid_arm_5dof_ros2
   cd ..
   ```

2. **Install dependencies:**
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace:**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

4. **Generate MoveIt configuration (first time only):**
   ```bash
   ros2 launch moveit_setup_assistant setup_assistant.launch.py
   # Load: src/humanoid_arm_5dof_ros2/humanoid_arm_description/urdf/humanoid_arm_5dof_ros2_control.urdf.xacro
   # Follow the setup wizard to generate humanoid_arm_moveit_config package
   ```

## 🎮 Usage Examples

### 1. Basic Robot Visualization

Launch robot with joint state publisher GUI:
```bash
ros2 launch humanoid_arm_description display.launch.py
```

### 2. Simulation with Controllers

Start robot with fake hardware:
```bash
ros2 launch humanoid_arm_bringup robot.launch.py
```

### 3. Gazebo Simulation

**Empty world (basic testing):**
```bash
ros2 launch humanoid_arm_bringup gazebo.launch.py world_file:=empty_world.sdf
```

**Contact manipulation arena:**
```bash
ros2 launch humanoid_arm_bringup gazebo.launch.py world_file:=contact_manipulation_arena.sdf
```

### 4. Joystick Teleoperation

Connect a joystick and launch teleoperation:
```bash
# Terminal 1: Start robot
ros2 launch humanoid_arm_bringup robot.launch.py

# Terminal 2: Start joystick teleop
ros2 launch humanoid_arm_teleop joystick_teleop.launch.py
```

**Joystick Controls (PlayStation controller):**
- **L1**: Deadman switch (must hold for motion)
- **R1**: Emergency stop
- **X**: Cycle control modes
- **Left stick**: Primary joint control
- **Right stick**: Secondary joint control
- **Circle**: Home position
- **Square**: Ready position

### 5. MoveIt2 Motion Planning

```bash
# Terminal 1: Start robot
ros2 launch humanoid_arm_bringup robot.launch.py

# Terminal 2: Start MoveIt
ros2 launch humanoid_arm_moveit_config demo.launch.py
```

## 🎯 Control Modes

### 1. Individual Joint Control
- Direct control of each joint independently
- Useful for fine-tuning and testing specific joints
- Maps joystick axes to joint velocities

### 2. End-Effector IK Control
- Cartesian control of arm tip position/orientation
- Real-time inverse kinematics solving
- Intuitive 6DOF control via joystick

### 3. Predefined Poses
- Quick access to common arm configurations
- Safe, tested poses for various tasks
- One-button activation

### 4. Whole-Arm Contact Mode
- Specialized for contact-based manipulation
- Force/torque feedback integration
- Compliant motion for safe contact

## 🌍 Simulation Environments

### Environment 1: Empty World
- **File**: `empty_world.sdf`
- **Purpose**: Basic testing and calibration
- **Features**: Flat ground, target spheres
- **Use cases**: Initial testing, joint calibration

### Environment 2: Contact Manipulation Arena
- **File**: `contact_manipulation_arena.sdf`
- **Purpose**: Whole-arm manipulation testing
- **Features**:
  - Various weighted objects (0.5kg - 5kg)
  - Elevated platforms
  - Inclined surfaces
  - Rolling ball
  - Obstacle barriers
- **Use cases**: Pushing, sliding, contact tasks

### Environment 3: Whole-Arm Workspace (TODO)
- **File**: `whole_arm_workspace.sdf`
- **Purpose**: Complex manipulation scenarios
- **Features**: Buttons, levers, doors, drawers
- **Use cases**: Multi-contact manipulation

## 🔧 Configuration

### Controller Configuration
Edit `humanoid_arm_control/config/controllers.yaml`:
- Joint trajectory controller settings
- Individual joint controllers
- Force-torque sensor configuration
- Safety limits and constraints

### Hardware Configuration
Edit `humanoid_arm_description/config/ros2_control.yaml`:
- Robstride motor specifications
- CAN interface settings
- Joint limits and dynamics
- Contact sensing parameters

### Actuator Simulation
Edit `humanoid_arm_control/config/actuator_specs.yaml`:
- Detailed motor specifications (RMD-X8-PRO, RMD-X6, RMD-X4)
- PID controller gains (position and velocity control)
- Motor dynamics (inertia, friction, back-EMF)
- Encoder specifications
- Communication parameters
- Thermal limits

### Teleoperation Configuration
Edit `humanoid_arm_teleop/config/joystick_config.yaml`:
- Button mappings
- Velocity scaling
- Control mode settings
- Safety parameters

## 🛠️ Hardware Integration

### Transition to Real Hardware

1. **Update hardware parameter:**
   ```bash
   ros2 launch humanoid_arm_bringup robot.launch.py use_fake_hardware:=false
   ```

2. **Configure CAN interface:**
   ```bash
   sudo ip link set can0 up type can bitrate 1000000
   ```

3. **Verify motor connections:**
   - Check CAN IDs match configuration
   - Verify motor types and specifications
   - Test individual motor communication

### Robstride Motor Specifications
- **Base rotation**: RMD-X8-PRO (120Nm, CAN ID: 0x01)
- **Shoulder pitch**: RMD-X8-PRO (120Nm, CAN ID: 0x02)
- **Elbow pitch**: RMD-X6 (60Nm, CAN ID: 0x03)
- **Wrist pitch**: RMD-X6 (60Nm, CAN ID: 0x04)
- **Wrist roll**: RMD-X4 (17Nm, CAN ID: 0x05)

## 🔍 Monitoring and Diagnostics

### Joint States
```bash
ros2 topic echo /joint_states
```

### Contact Forces
```bash
ros2 topic echo /humanoid_arm/tcp_wrench
ros2 topic echo /humanoid_arm/forearm_wrench
ros2 topic echo /humanoid_arm/wrist_wrench
```

### Controller Status
```bash
ros2 control list_controllers
ros2 control switch_controllers --start-controllers joint_trajectory_controller
```

### IMU Data
```bash
ros2 topic echo /humanoid_arm/arm_imu/data
```

## 🐛 Troubleshooting

### Common Issues

**1. Controllers not loading:**
```bash
# Check controller manager status
ros2 control list_controllers

# Manually spawn controllers
ros2 run controller_manager spawner joint_trajectory_controller
```

**2. Gazebo simulation slow:**
- Reduce physics update rate in world file
- Close unnecessary GUI panels
- Use headless mode: `gui:=false`

**3. Joystick not detected:**
```bash
# List available joysticks
ls /dev/input/js*

# Test joystick
jstest /dev/input/js0

# Give permissions
sudo chmod 666 /dev/input/js0
```

**4. MoveIt planning fails:**
- Check joint limits in URDF
- Verify planning group configuration
- Increase planning time limits

### Debug Commands

```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Monitor joint limits
ros2 param get /controller_manager joint_limits

# View robot model
ros2 launch robot_state_publisher view_robot.launch.py model:=src/humanoid_arm_description/urdf/humanoid_arm_5dof.urdf
```

## 📊 Performance Notes

### Recommended Specifications
- **CPU**: Intel i5 or AMD Ryzen 5 (minimum)
- **RAM**: 8GB minimum, 16GB recommended
- **GPU**: Dedicated graphics recommended for Gazebo
- **Real-time kernel**: Recommended for hardware control

### Latency Requirements
- **Simulation**: 1ms physics timestep
- **Hardware control**: 1-10ms control loop
- **Teleoperation**: 50Hz update rate
- **Force feedback**: 100Hz for contact sensing

## 🤝 Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

## 📜 License

This project is licensed under the BSD-3-Clause License - see the LICENSE file for details.

## 🙏 Acknowledgments

- **SolidWorks to URDF Exporter** for initial robot model
- **ROS2 Control** framework for hardware abstraction
- **MoveIt2** for motion planning capabilities
- **Gazebo** for realistic physics simulation

## 📞 Support

For issues and questions:
- Create an issue on GitHub
- Check existing documentation
- Review troubleshooting section

---

**Note**: This is a simulation-focused development package. Hardware integration requires proper safety measures and testing protocols.
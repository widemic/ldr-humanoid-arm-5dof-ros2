# Installation Guide - Humanoid Arm 5-DOF ROS2

Complete installation instructions for setting up the 5-DOF humanoid arm ROS2 workspace on Ubuntu 24.04 with ROS2 Jazzy. **All components are installed from binary packages (apt)** for stability, speed, and ease of maintenance.

## Table of Contents

1. [System Requirements](#system-requirements)
2. [Why Install from Packages?](#why-install-from-packages)
3. [ROS2 Jazzy Installation](#ros2-jazzy-installation)
4. [Gazebo Harmonic Installation](#gazebo-harmonic-installation)
5. [ROS2 Control and Controllers](#ros2-control-and-controllers)
6. [MoveIt2 Installation](#moveit2-installation)
7. [Additional Dependencies](#additional-dependencies)
8. [Python Dependencies](#python-dependencies)
9. [Workspace Setup](#workspace-setup)
10. [Verification](#verification)
11. [Optional Tools](#optional-tools)
12. [Troubleshooting](#troubleshooting)

---

## System Requirements

### Hardware
- **CPU**: Intel i5/AMD Ryzen 5 or better (quad-core minimum)
- **RAM**: 8GB minimum, 16GB recommended
- **GPU**: Dedicated graphics recommended for Gazebo simulation
- **Storage**: 10GB free space (no source builds needed)
- **Joystick**: PlayStation or Xbox controller (optional, for teleoperation)

### Software
- **OS**: Ubuntu 24.04 (Noble Numbat) - **Required for ROS2 Jazzy**
- **Kernel**: Standard or real-time kernel (real-time recommended for hardware control)

---

## Why Install from Packages?

This guide uses **binary packages (apt)** exclusively for ROS2, ros2_control, MoveIt2, and all dependencies. **Do not build from source unless you have a specific need.**

### Advantages of Package Installation

✅ **Fast setup** - No compilation time (saves 2-4 hours)
✅ **Stable releases** - Tested, versioned packages matched to ROS2 Jazzy
✅ **Easy updates** - `sudo apt upgrade` keeps everything current
✅ **Automatic dependencies** - No manual dependency management
✅ **Less disk space** - ~5GB vs ~25GB for source builds
✅ **Industry standard** - Most teams use binary packages for development

### When to Build from Source (Not This Project)

Only consider source builds if:
- Contributing patches to ros2_control/MoveIt upstream
- Debugging core framework bugs
- Needing unreleased features not in stable release
- Modifying framework internals

**For this project:** You're developing custom arm control, not modifying ROS2 internals. Package installation is the right choice.

---

## ROS2 Jazzy Installation

### 1. Set Locale
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 2. Add ROS2 APT Repository
```bash
# Add the ROS 2 GPG key
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 3. Install ROS2 Jazzy Desktop Full
```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop-full
```

**Packages included:**
- ROS2 core libraries
- rclcpp, rclpy (C++ and Python client libraries)
- RViz2 (visualization)
- Robot State Publisher
- TF2 (coordinate frame management)
- Demos and tutorials

### 4. Install Development Tools
```bash
sudo apt install ros-dev-tools
```

**Includes:**
- `colcon` (build tool)
- `rosdep` (dependency manager)
- `vcstool` (version control)

### 5. Source ROS2 (Add to ~/.bashrc)
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Gazebo Harmonic Installation

This project uses **Gazebo Harmonic** (Gazebo Sim 8), not Gazebo Classic.

### 1. Install Gazebo Harmonic
```bash
sudo apt-get update
sudo apt-get install ros-jazzy-ros-gz-sim
```

**This package includes:**
- `gz-sim8` (Gazebo Harmonic simulator)
- ROS2-Gazebo simulation bridge
- Launch file integration

**Additional Gazebo packages:**
```bash
sudo apt install ros-jazzy-ros-gz-bridge
sudo apt install ros-jazzy-ros-gz-image
```

**Provides:**
- `ros-jazzy-ros-gz-bridge` (topic/service bridge)
- `ros-jazzy-ros-gz-image` (camera/image transport)

### 2. Install Gazebo ROS2 Control Plugin
```bash
sudo apt install ros-jazzy-gz-ros2-control
```

**Required for:**
- Hardware interface integration with Gazebo
- Effort-based control in simulation
- Joint state feedback from Gazebo

### 3. Verify Gazebo Installation
```bash
gz sim --version
# Should output: Gazebo Sim, version 8.x.x
```

---

## ROS2 Control and Controllers

### 1. Install ros2_control Framework
```bash
sudo apt install ros-jazzy-ros2-control
```

**Provides:**
- `controller_manager`
- `hardware_interface`
- `controller_interface`
- Hardware abstraction layer

### 2. Install ros2_controllers
```bash
sudo apt install ros-jazzy-ros2-controllers
```

**Includes:**
- `joint_trajectory_controller` (primary for MoveIt2)
- `joint_state_broadcaster`
- `position_controllers` (forward_command, joint_group)
- `velocity_controllers`
- `effort_controllers`
- `imu_sensor_broadcaster`
- `force_torque_sensor_broadcaster`

### 3. Install Mock Components (for testing without hardware)
```bash
sudo apt install ros-jazzy-ros2-control-test-assets
```

---

## MoveIt2 Installation

### 1. Install MoveIt2 for Jazzy
```bash
sudo apt install ros-jazzy-moveit
```

**Includes:**
- `moveit_ros_move_group` (motion planning node)
- `moveit_planners` (OMPL, Pilz, CHOMP)
- `moveit_kinematics` (KDL, TracIK solvers)
- `moveit_simple_controller_manager`
- `moveit_ros_visualization` (RViz plugins)
- `moveit_ros_warehouse` (database support)

### 2. Install MoveIt Setup Assistant
```bash
sudo apt install ros-jazzy-moveit-setup-assistant
```

### 3. Install MoveIt Servo (for teleoperation)
```bash
sudo apt install ros-jazzy-moveit-servo
```

### 4. Install Additional MoveIt Utilities
```bash
sudo apt install ros-jazzy-moveit-configs-utils
sudo apt install ros-jazzy-warehouse-ros-mongo
```

---

## Additional Dependencies

### 1. URDF and Xacro Tools
```bash
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-urdf
sudo apt install ros-jazzy-joint-state-publisher
sudo apt install ros-jazzy-joint-state-publisher-gui
```

### 2. Joystick/Teleoperation Support
```bash
sudo apt install ros-jazzy-joy
sudo apt install ros-jazzy-teleop-twist-joy
sudo apt install joystick jstest-gtk
```

**Configure joystick permissions:**
```bash
sudo usermod -a -G input $USER
# Log out and log back in for group changes to take effect
```

### 3. TF2 Utilities
```bash
sudo apt install ros-jazzy-tf2-tools
sudo apt install ros-jazzy-tf2-geometry-msgs
```

### 4. RViz2 Additional Plugins
```bash
sudo apt install ros-jazzy-rviz-common
sudo apt install ros-jazzy-rviz-default-plugins
```

### 5. Gazebo Additional Plugins
```bash
sudo apt install ros-jazzy-gazebo-plugins
sudo apt install ros-jazzy-gazebo-ros2-control
```

---

## Python Dependencies

### 1. Install Python3 and Pip
```bash
sudo apt install python3-pip python3-dev
```

### 2. Install ROS2 Python Libraries
```bash
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
sudo apt install python3-vcstool
```

### 3. Install Python Scientific Libraries
```bash
# Install from apt (preferred for system integration)
sudo apt install python3-tk python3-matplotlib python3-numpy python3-scipy
```

**Why apt instead of pip for these?**
- Better integration with ROS2 Python libraries
- No virtual environment conflicts
- Consistent with binary package philosophy

---

## Workspace Setup

### 1. Create ROS2 Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone the Repository
```bash
git clone https://github.com/your-username/ldr-humanoid-arm-5dof-ros2.git
cd ~/ros2_ws
```

### 3. Initialize rosdep (First Time Only)
```bash
sudo rosdep init
rosdep update
```

### 4. Install Package Dependencies
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

**This command automatically installs all dependencies listed in package.xml files.**

### 5. Build the Workspace
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

**Build flags explained:**
- `--symlink-install`: Creates symlinks instead of copying files (faster iteration)
- Add `--packages-select <package_name>` to build specific packages
- Add `--event-handlers console_direct+` for verbose build output

### 6. Source the Workspace
```bash
source ~/ros2_ws/install/setup.bash
```

**Add to ~/.bashrc for automatic sourcing:**
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Verification

### 1. Verify ROS2 Installation
```bash
ros2 --version
# Should output: ros2 version jazzy
```

### 2. Verify Gazebo Integration
```bash
gz sim --version
ros2 pkg list | grep gazebo
```

### 3. Test Robot Model Loading
```bash
ros2 launch humanoid_arm_description display.launch.py
```

**Expected result:** RViz opens with robot model and joint sliders

### 4. Test Mock Hardware Mode
```bash
ros2 launch humanoid_arm_bringup robot.launch.py
```

**Verify controllers are loaded:**
```bash
ros2 control list_controllers
```

**Expected output:**
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active
```

### 5. Test Gazebo Simulation
```bash
ros2 launch humanoid_arm_bringup gazebo.launch.py
```

**Expected result:** Gazebo opens with robot in empty world

### 6. Test Full System (Gazebo + MoveIt + RViz)
```bash
ros2 launch humanoid_arm_bringup full_system.launch.py
```

**Expected result:** All three windows open (Gazebo, RViz with MoveIt plugin)

---

## Optional Tools

### 1. Real-Time Kernel (for Hardware Control)
```bash
# Check available RT kernels
apt-cache search linux-image-rt

# Install RT kernel (example)
sudo apt install linux-image-rt-amd64
sudo reboot

# Verify RT kernel
uname -a | grep PREEMPT_RT
```

### 2. CAN Bus Tools (for Robstride Motors)
```bash
sudo apt install can-utils
```

**Configure CAN interface:**
```bash
sudo ip link set can0 up type can bitrate 1000000
```

### 3. Diagnostic Tools
```bash
sudo apt install ros-jazzy-rqt
sudo apt install ros-jazzy-rqt-common-plugins
sudo apt install ros-jazzy-rqt-joint-trajectory-controller
sudo apt install ros-jazzy-plotjuggler-ros
```

**Usage:**
```bash
# Launch RQt with plugins (topic viewer, service caller, parameter editor, etc.)
rqt

# Launch PlotJuggler for real-time data plotting
ros2 run plotjuggler plotjuggler
```

### 4. Code Development Tools
```bash
# VS Code with ROS extension
sudo snap install --classic code

# C++ development tools
sudo apt install build-essential clang-format clangd
sudo apt install ros-jazzy-ament-clang-format
sudo apt install ros-jazzy-ament-cmake

# Python development tools
sudo apt install python3-pylint python3-autopep8
```

**Note:** All development tools installed via apt for consistency.

---

## Troubleshooting

### Issue: "Package 'ros-jazzy-<package>' has no installation candidate"

**Solution:**
```bash
sudo apt update
sudo apt upgrade
rosdep update
```

### Issue: Gazebo fails to start with "gz-sim: command not found"

**Solution:**
```bash
# Verify Gazebo installation
dpkg -l | grep gz-sim

# Reinstall if missing
sudo apt install ros-jazzy-ros-gz
```

### Issue: Controllers fail to load with "resource not found"

**Solution:**
```bash
# Rebuild workspace
cd ~/ros2_ws
colcon build --symlink-install --packages-select humanoid_arm_control
source install/setup.bash
```

### Issue: rosdep install fails

**Solution:**
```bash
# Reinitialize rosdep
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
rosdep update
```

### Issue: Joystick not detected

**Solution:**
```bash
# Check device
ls /dev/input/js*

# Test joystick
jstest /dev/input/js0

# Fix permissions
sudo chmod 666 /dev/input/js0
sudo usermod -a -G input $USER
# Log out and back in
```

### Issue: Import error for Python modules

**Solution:**
```bash
# Install missing Python dependencies (use apt, not pip)
sudo apt install python3-numpy python3-scipy python3-matplotlib python3-tk

# Verify Python path includes ROS2
python3 -c "import rclpy; print(rclpy.__file__)"
```

### Issue: Build fails with "Could not find a package configuration file"

**Solution:**
```bash
# Check for missing dependencies
rosdep check --from-paths src --ignore-src

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y
```

---

## Complete Installation Script

For convenience, here's a complete installation script (Ubuntu 24.04 + ROS2 Jazzy). **All packages installed from apt - no source builds.**

```bash
#!/bin/bash

# Exit on error
set -e

echo "Installing ROS2 Jazzy and Humanoid Arm Dependencies (from binary packages)..."

# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 Jazzy
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-desktop-full -y
sudo apt install ros-dev-tools -y

# Install Gazebo Harmonic
sudo apt install ros-jazzy-ros-gz-sim -y
sudo apt install ros-jazzy-ros-gz-bridge -y
sudo apt install ros-jazzy-ros-gz-image -y
sudo apt install ros-jazzy-gz-ros2-control -y

# Install ros2_control and controllers
sudo apt install ros-jazzy-ros2-control -y
sudo apt install ros-jazzy-ros2-controllers -y
sudo apt install ros-jazzy-ros2-control-test-assets -y

# Install MoveIt2
sudo apt install ros-jazzy-moveit -y
sudo apt install ros-jazzy-moveit-setup-assistant -y
sudo apt install ros-jazzy-moveit-servo -y
sudo apt install ros-jazzy-moveit-configs-utils -y
sudo apt install ros-jazzy-warehouse-ros-mongo -y

# Install additional tools
sudo apt install ros-jazzy-xacro -y
sudo apt install ros-jazzy-urdf -y
sudo apt install ros-jazzy-joint-state-publisher -y
sudo apt install ros-jazzy-joint-state-publisher-gui -y
sudo apt install ros-jazzy-joy -y
sudo apt install ros-jazzy-teleop-twist-joy -y
sudo apt install joystick jstest-gtk -y
sudo apt install ros-jazzy-tf2-tools -y
sudo apt install ros-jazzy-gazebo-plugins -y
sudo apt install ros-jazzy-rqt -y
sudo apt install ros-jazzy-rqt-common-plugins -y

# Install Python dependencies (all from apt)
sudo apt install python3-pip python3-dev python3-tk python3-matplotlib python3-numpy python3-scipy -y

# Initialize rosdep
sudo rosdep init || true
rosdep update

# Add ROS2 to bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Configure joystick permissions
sudo usermod -a -G input $USER

echo ""
echo "=========================================="
echo "Installation complete!"
echo "=========================================="
echo ""
echo "All components installed from binary packages (apt):"
echo "  ✓ ROS2 Jazzy Desktop Full"
echo "  ✓ Gazebo Harmonic with ROS2 integration"
echo "  ✓ ros2_control and controllers"
echo "  ✓ MoveIt2 with all dependencies"
echo "  ✓ Development tools and Python libraries"
echo ""
echo "Next steps:"
echo "  1. Log out and log back in (for input group permissions)"
echo "  2. Create workspace: mkdir -p ~/ros2_ws/src"
echo "  3. Clone repository: cd ~/ros2_ws/src && git clone <repository-url>"
echo "  4. Install package deps: cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -r -y"
echo "  5. Build workspace: colcon build --symlink-install"
echo "  6. Source workspace: source install/setup.bash"
echo ""
echo "Total installation time: ~10-15 minutes (vs 2-4 hours for source builds)"
```

**Save as `install_ros2_jazzy.sh` and run:**
```bash
chmod +x install_ros2_jazzy.sh
./install_ros2_jazzy.sh
```

**Installation time:** ~10-15 minutes (all binary packages, no compilation)

---

## Next Steps

After successful installation:

1. **Read the main README.md** for usage examples and architecture overview
2. **Read PID_README.md** for PID tuning guide (critical for effort control)
3. **Read CLAUDE.md** for development workflow and tips
4. **Test each launch mode** (mock hardware, Gazebo, full system)
5. **Explore simulation worlds** in `humanoid_arm_bringup/worlds/`

---

## Installation Philosophy Summary

This project uses **binary packages exclusively** for:
- ✅ **Speed**: 10-15 min install vs 2-4 hours
- ✅ **Stability**: Tested, versioned releases
- ✅ **Maintainability**: Simple `apt upgrade` updates
- ✅ **Focus**: Spend time developing arm control, not debugging ROS2 builds

**Your workspace only builds your custom packages:**
- `humanoid_arm_description`
- `humanoid_arm_control`
- `humanoid_arm_moveit_config`
- `humanoid_arm_bringup`
- `humanoid_arm_teleop`

**ROS2/ros2_control/MoveIt2 stay as binary packages.** This is the recommended approach for application development.

---

## Support

If you encounter issues not covered in troubleshooting:

1. Check ROS2 Jazzy documentation: https://docs.ros.org/en/jazzy/
2. Check Gazebo Harmonic docs: https://gazebosim.org/docs/harmonic
3. Review package-specific issues in GitHub repository
4. Verify Ubuntu 24.04 compatibility for all packages

**Important:** This project requires ROS2 Jazzy on Ubuntu 24.04. Older ROS2 distributions (Humble, Iron) or Ubuntu versions may not work correctly.

**Binary packages only:** Do not mix source builds with binary packages unless absolutely necessary.

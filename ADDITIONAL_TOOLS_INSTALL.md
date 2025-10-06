# Additional Tools and Dependencies Installation

Installation guide for Gazebo, diagnostic tools, Python packages, and other utilities not included in the core ROS2, ros2_control, and MoveIt2 installations.

## Prerequisites

- **ROS2 Jazzy** installed - See [ROS2_INSTALL.md](ROS2_INSTALL.md)
- **ros2_control** installed - See [ROS2_CONTROL_INSTALL.md](ROS2_CONTROL_INSTALL.md)
- **MoveIt2** installed - See [MOVEIT2_INSTALL.md](MOVEIT2_INSTALL.md)
- **Ubuntu 24.04** (Noble Numbat)

---

## Table of Contents

1. [Gazebo Harmonic (Simulation)](#1-gazebo-harmonic-simulation)
2. [ROS-Gazebo Integration](#2-ros-gazebo-integration)
3. [URDF and Visualization Tools](#3-urdf-and-visualization-tools)
4. [Joystick and Teleoperation](#4-joystick-and-teleoperation)
5. [Diagnostic and Monitoring Tools](#5-diagnostic-and-monitoring-tools)
6. [Python Dependencies](#6-python-dependencies)
7. [Development Tools](#7-development-tools)
8. [Real-Time Kernel (Hardware)](#8-real-time-kernel-hardware)
9. [CAN Bus Tools (Robstride Motors)](#9-can-bus-tools-robstride-motors)
10. [Optional Utilities](#10-optional-utilities)

---

## 1. Gazebo Harmonic (Simulation)

This project uses **Gazebo Harmonic** (Gazebo Sim 8), not Gazebo Classic.

### Install Gazebo Harmonic

```bash
# Add Gazebo repository
sudo apt-get update
sudo apt-get install wget
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update and install Gazebo Harmonic
sudo apt-get update
sudo apt-get install gz-harmonic
```

### Verify Installation

```bash
gz sim --version
# Should output: Gazebo Sim, version 8.x.x
```

### Test Gazebo

```bash
# Launch empty world
gz sim empty.sdf

# Or with shapes demo
gz sim shapes.sdf
```

---

## 2. ROS-Gazebo Integration

### Install ROS2-Gazebo Bridge

```bash
sudo apt install ros-jazzy-ros-gz-sim
sudo apt install ros-jazzy-ros-gz-bridge
sudo apt install ros-jazzy-ros-gz-image
```

**Provides:**
- ROS2 topic/service bridge to Gazebo
- Camera and sensor integration
- Launch file utilities

### Install Gazebo ros2_control Plugin

```bash
sudo apt install ros-jazzy-gz-ros2-control
```

**Critical for this project:**
- Hardware interface for Gazebo
- Effort-based control (realistic dynamics)
- Joint state feedback

### Verify Integration

```bash
# Check packages
ros2 pkg list | grep gz

# Expected output:
# ros_gz_bridge
# ros_gz_image
# ros_gz_sim
# gz_ros2_control
```

---

## 3. URDF and Visualization Tools

### URDF/Xacro Tools

```bash
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-urdf
sudo apt install liburdfdom-tools
```

**Provides:**
- `xacro` - Macro processor for URDF
- `check_urdf` - URDF validator
- `urdf_to_graphviz` - Visualize link tree

### Robot State Publisher

```bash
sudo apt install ros-jazzy-robot-state-publisher
sudo apt install ros-jazzy-joint-state-publisher
sudo apt install ros-jazzy-joint-state-publisher-gui
```

### RViz2 and Plugins

```bash
sudo apt install ros-jazzy-rviz2
sudo apt install ros-jazzy-rviz-common
sudo apt install ros-jazzy-rviz-default-plugins
```

### TF2 Tools

```bash
sudo apt install ros-jazzy-tf2-tools
sudo apt install ros-jazzy-tf2-ros
sudo apt install ros-jazzy-tf2-geometry-msgs
```

**Usage:**
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo world base_link
```

---

## 4. Joystick and Teleoperation

### Joystick Support

```bash
# ROS2 joy package
sudo apt install ros-jazzy-joy
sudo apt install ros-jazzy-teleop-twist-joy

# Linux joystick tools
sudo apt install joystick jstest-gtk
```

### Configure Joystick Permissions

```bash
# Add user to input group
sudo usermod -a -G input $USER

# Log out and log back in for changes to take effect
```

### Test Joystick

```bash
# List joysticks
ls /dev/input/js*

# Test joystick input
jstest /dev/input/js0

# Or use GUI
jstest-gtk
```

### Set Joystick Permissions (if needed)

```bash
# Temporary (until reboot)
sudo chmod 666 /dev/input/js0

# Permanent (create udev rule)
sudo bash -c 'cat > /etc/udev/rules.d/99-joystick.rules << EOF
KERNEL=="js*", MODE="0666"
EOF'

sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Test with ROS2

```bash
# Terminal 1: Start joy node
ros2 run joy joy_node

# Terminal 2: Echo joy topic
ros2 topic echo /joy
# Press buttons on joystick to see output
```

---

## 5. Diagnostic and Monitoring Tools

### RQt and Plugins

```bash
# Core RQt
sudo apt install ros-jazzy-rqt
sudo apt install ros-jazzy-rqt-common-plugins

# Additional useful plugins
sudo apt install ros-jazzy-rqt-joint-trajectory-controller
sudo apt install ros-jazzy-rqt-controller-manager
sudo apt install ros-jazzy-rqt-robot-monitor
sudo apt install ros-jazzy-rqt-robot-steering
```

**RQt Plugins Include:**
- Topic viewer/publisher
- Service caller
- Parameter editor
- Console (log viewer)
- Graph (node/topic visualization)
- Plot (real-time data plotting)
- Image viewer
- TF tree

**Usage:**
```bash
# Launch RQt
rqt

# Or specific plugin
rqt_graph
rqt_console
rqt_plot
```

### PlotJuggler (Advanced Plotting)

```bash
sudo apt install ros-jazzy-plotjuggler-ros
```

**Usage:**
```bash
ros2 run plotjuggler plotjuggler

# Or with ROS2 plugins
ros2 run plotjuggler plotjuggler --ros2
```

**Features:**
- Real-time data plotting
- Time series analysis
- Topic recording/playback
- Export to CSV

### Qt Designer (for GUI development)

```bash
sudo apt install qttools5-dev-tools
sudo apt install python3-pyqt5
```

**Usage:**
```bash
# Launch Qt Designer
designer

# Open .ui file
designer humanoid_arm_control/ui/trajectory_tuner.ui
```

---

## 6. Python Dependencies

### Core Python Tools

```bash
sudo apt install python3-pip python3-dev python3-venv
```

### Python GUI Libraries

```bash
# Tkinter (for PID tuner GUI)
sudo apt install python3-tk

# Matplotlib (plotting)
sudo apt install python3-matplotlib

# PyQt5 (Qt bindings)
sudo apt install python3-pyqt5
```

### Scientific Computing

```bash
# NumPy and SciPy
pip3 install --user numpy scipy

# Or via apt (may be older versions)
sudo apt install python3-numpy python3-scipy
```

### ROS2 Python Tools

```bash
# Python colcon extensions
sudo apt install python3-colcon-common-extensions

# Python rosdep
sudo apt install python3-rosdep python3-rosdep2

# Python vcstool
sudo apt install python3-vcstool
```

---

## 7. Development Tools

### C++ Development

```bash
# Build essentials
sudo apt install build-essential cmake

# Clang tools (formatting, linting)
sudo apt install clang-format clangd
sudo apt install ros-jazzy-ament-clang-format
sudo apt install ros-jazzy-ament-clang-tidy

# GDB debugger
sudo apt install gdb
```

### Python Development

```bash
# Linting and formatting
pip3 install --user pylint black flake8

# ROS2 Python linters
sudo apt install python3-flake8
sudo apt install ros-jazzy-ament-flake8
sudo apt install ros-jazzy-ament-pep257
```

### VS Code with ROS Extension

```bash
# Install VS Code
sudo snap install --classic code

# Install ROS extension (from within VS Code):
# Search for "ROS" and install the Microsoft ROS extension
```

**Recommended VS Code Extensions:**
- ROS (Microsoft)
- C/C++ (Microsoft)
- Python (Microsoft)
- CMake Tools
- XML Tools (for URDF/launch files)

### Git Tools

```bash
sudo apt install git gitk git-gui
```

---

## 8. Real-Time Kernel (Hardware)

For real hardware control, a real-time kernel is highly recommended.

### Check Current Kernel

```bash
uname -a
# Check for PREEMPT_RT in output
```

### Install Real-Time Kernel

```bash
# Search for available RT kernels
apt-cache search linux-image-rt

# Install (example for Ubuntu 24.04)
sudo apt install linux-image-rt-amd64 linux-headers-rt-amd64

# Or for specific version
sudo apt install linux-image-$(uname -r)-rt
```

### Configure Real-Time Permissions

```bash
# Add user to realtime group
sudo groupadd realtime
sudo usermod -a -G realtime $USER

# Set limits
sudo bash -c 'cat > /etc/security/limits.d/99-realtime.conf << EOF
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
EOF'

# Log out and log back in
```

### Reboot and Select RT Kernel

```bash
sudo reboot

# After reboot, verify
uname -a | grep PREEMPT_RT
```

### Set Controller Priority (with RT kernel)

```bash
# Allow controller_manager to use real-time priority
sudo setcap cap_sys_nice+ep ~/ros2_control_ws/install/controller_manager/lib/controller_manager/ros2_control_node
```

---

## 9. CAN Bus Tools (Robstride Motors)

For hardware integration with Robstride RMD-X motors.

### Install CAN Utilities

```bash
sudo apt install can-utils
```

**Provides:**
- `candump` - Display CAN messages
- `cansend` - Send CAN messages
- `canconfig` - Configure CAN interface
- `cangen` - Generate random CAN traffic (testing)

### Configure CAN Interface

```bash
# Bring up CAN interface (1 Mbps bitrate for Robstride)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# Verify
ip -details link show can0
```

### Test CAN Interface

```bash
# Terminal 1: Monitor CAN traffic
candump can0

# Terminal 2: Send test message
cansend can0 123#DEADBEEF
```

### Persistent CAN Configuration

Create `/etc/systemd/network/80-can.network`:

```bash
sudo bash -c 'cat > /etc/systemd/network/80-can.network << EOF
[Match]
Name=can0

[CAN]
BitRate=1000000
EOF'

sudo systemctl restart systemd-networkd
```

### Install Robstride SDK (Future)

```bash
# Clone Robstride SDK (when available)
# cd ~/ros2_ws/src
# git clone https://github.com/Robstride/robstride_ros2.git
```

---

## 10. Optional Utilities

### Bag Recording/Playback

```bash
sudo apt install ros-jazzy-rosbag2
sudo apt install ros-jazzy-rosbag2-storage-default-plugins
```

**Usage:**
```bash
# Record topics
ros2 bag record -a  # Record all topics
ros2 bag record /joint_states /tf

# Playback
ros2 bag play <bag_file>
```

### Documentation Tools

```bash
# Doxygen (C++ documentation)
sudo apt install doxygen graphviz

# Sphinx (Python documentation)
pip3 install --user sphinx sphinx-rtd-theme
```

### Network Tools

```bash
# Monitor network traffic
sudo apt install iftop nethogs

# Network diagnostics
sudo apt install net-tools iputils-ping
```

### System Monitoring

```bash
# htop (process monitor)
sudo apt install htop

# iotop (I/O monitor)
sudo apt install iotop

# CPU frequency tools
sudo apt install cpufrequtils
```

### Terminal Multiplexer

```bash
# tmux or screen for multiple terminals
sudo apt install tmux

# Or screen
sudo apt install screen
```

---

## Complete Installation Script

For convenience, here's a script to install all additional tools:

```bash
#!/bin/bash
set -e

echo "Installing Additional Tools for Humanoid Arm Project..."

# Gazebo Harmonic
echo "Installing Gazebo Harmonic..."
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install -y gz-harmonic

# ROS-Gazebo Integration
echo "Installing ROS-Gazebo integration..."
sudo apt install -y \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image \
  ros-jazzy-gz-ros2-control

# URDF and Visualization
echo "Installing URDF and visualization tools..."
sudo apt install -y \
  ros-jazzy-xacro \
  ros-jazzy-urdf \
  liburdfdom-tools \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-rviz-common \
  ros-jazzy-rviz-default-plugins \
  ros-jazzy-tf2-tools \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs

# Joystick and Teleoperation
echo "Installing joystick support..."
sudo apt install -y \
  ros-jazzy-joy \
  ros-jazzy-teleop-twist-joy \
  joystick \
  jstest-gtk

# Diagnostic Tools
echo "Installing diagnostic tools..."
sudo apt install -y \
  ros-jazzy-rqt \
  ros-jazzy-rqt-common-plugins \
  ros-jazzy-rqt-joint-trajectory-controller \
  ros-jazzy-rqt-controller-manager \
  ros-jazzy-plotjuggler-ros

# Python Dependencies
echo "Installing Python dependencies..."
sudo apt install -y \
  python3-pip \
  python3-dev \
  python3-tk \
  python3-matplotlib \
  python3-pyqt5 \
  python3-numpy \
  python3-scipy

# Development Tools
echo "Installing development tools..."
sudo apt install -y \
  build-essential \
  cmake \
  clang-format \
  clangd \
  gdb \
  qttools5-dev-tools

# CAN Bus Tools
echo "Installing CAN bus tools..."
sudo apt install -y can-utils

# Optional Utilities
echo "Installing optional utilities..."
sudo apt install -y \
  ros-jazzy-rosbag2 \
  htop \
  tmux

# Configure joystick permissions
echo "Configuring joystick permissions..."
sudo usermod -a -G input $USER

echo ""
echo "Installation complete!"
echo ""
echo "IMPORTANT: Log out and log back in for group changes (joystick) to take effect."
echo ""
echo "Next steps:"
echo "1. Test Gazebo: gz sim empty.sdf"
echo "2. Test joystick: jstest /dev/input/js0"
echo "3. Build project workspace: cd ~/ros2_ws && colcon build"
```

**Save as `install_additional_tools.sh` and run:**
```bash
chmod +x install_additional_tools.sh
./install_additional_tools.sh
```

---

## Verification Checklist

After installation, verify each component:

- [ ] **Gazebo**: `gz sim --version` shows version 8.x.x
- [ ] **ROS-Gazebo**: `ros2 pkg list | grep gz` shows packages
- [ ] **URDF tools**: `check_urdf --help` works
- [ ] **Joystick**: `jstest /dev/input/js0` detects controller
- [ ] **RQt**: `rqt` launches GUI
- [ ] **PlotJuggler**: `ros2 run plotjuggler plotjuggler` launches
- [ ] **Python**: `python3 -c "import numpy, scipy, matplotlib"` works
- [ ] **TF tools**: `ros2 run tf2_tools --help` works
- [ ] **CAN utils**: `candump --help` works

---

## Next Steps

After installing all dependencies:

1. **Build project workspace** - See [README.md](README.md#installation)
2. **Test simulation modes** - See [CLAUDE.md](CLAUDE.md#launch-modes)
3. **Configure PID gains** - See [PID_README.md](PID_README.md)
4. **Test joystick control** - See [README.md](README.md#4-joystick-teleoperation)

---

## Troubleshooting

### Gazebo Issues

**Problem:** `gz sim` not found

**Solution:**
```bash
# Check if Gazebo is in PATH
which gz

# Add to PATH if needed
echo 'export PATH=$PATH:/usr/local/bin' >> ~/.bashrc
source ~/.bashrc
```

**Problem:** Gazebo crashes on startup

**Solution:**
```bash
# Check graphics drivers
glxinfo | grep "OpenGL"

# Update graphics drivers
sudo apt install mesa-utils
```

### Joystick Issues

**Problem:** `/dev/input/js0` not found

**Solution:**
```bash
# Check if joystick is connected
lsusb

# Check dmesg for joystick detection
dmesg | grep -i joy

# Verify input group membership
groups $USER
```

### CAN Bus Issues

**Problem:** `can0: No such device`

**Solution:**
```bash
# Check if CAN hardware exists
ip link show

# Load CAN kernel modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan

# Create virtual CAN (for testing without hardware)
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

---

## References

- Gazebo Harmonic: https://gazebosim.org/docs/harmonic
- ROS-Gazebo Integration: https://github.com/gazebosim/ros_gz
- RQt: https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-RQt.html
- PlotJuggler: https://github.com/facontidavide/PlotJuggler
- CAN Utils: https://github.com/linux-can/can-utils

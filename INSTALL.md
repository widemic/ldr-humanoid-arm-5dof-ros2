# Installation Guide

Complete setup instructions for the humanoid arm ROS2 workspace.

## Prerequisites

- Ubuntu 24.04 (Noble) - required for ROS2 Jazzy
- Internet connection
- sudo privileges

## Quick Installation (Recommended)

**One-command install for everything:**

```bash
# Install ROS2 Jazzy and all major dependencies
sudo ./install_ros2_dependencies.sh

# Build the workspace
./setup_workspace.sh

# Source and test
source install/setup.bash
ros2 launch humanoid_arm_description display.launch.py
```

That's it! The automated scripts handle all steps below.

---

## Manual Installation (Alternative)

If you prefer step-by-step installation or need to customize:

### Step 1: Install ROS2 Jazzy

```bash
# Add ROS2 apt repository
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy Desktop Full
sudo apt update
sudo apt install ros-jazzy-desktop-full
```

### Step 2: Install Development Tools

```bash
# Colcon build tool and rosdep dependency manager
sudo apt install python3-colcon-common-extensions python3-rosdep

# Initialize rosdep (first time only)
sudo rosdep init
rosdep update
```

### Step 3: Install Major Dependencies

```bash
# ros2_control and controllers
sudo apt install ros-jazzy-ros2-control \
                 ros-jazzy-ros2-controllers \
                 ros-jazzy-controller-manager

# Gazebo Harmonic and ROS2 integration
sudo apt install ros-jazzy-ros-gz-sim \
                 ros-jazzy-ros-gz-bridge \
                 ros-jazzy-gz-ros2-control

# MoveIt2 motion planning
sudo apt install ros-jazzy-moveit

# Additional utilities
sudo apt install ros-jazzy-joint-state-publisher-gui \
                 ros-jazzy-xacro \
                 ros-jazzy-robot-state-publisher
```

### Step 4: Install Workspace-Specific Dependencies

```bash
# Navigate to workspace root
cd ~/Documents/GitHub/ldr-humanoid-arm-5dof-ros2

# Install all dependencies from package.xml files
rosdep install --from-paths src --ignore-src -r -y
```

## Step 5: Build the Workspace

```bash
# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Step 6: Add to Shell Startup (Optional)

Add to `~/.bashrc` for automatic sourcing:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/Documents/GitHub/ldr-humanoid-arm-5dof-ros2/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Verify Installation

```bash
# Check ROS2 installation
ros2 --version

# List available packages
ros2 pkg list | grep humanoid

# Launch visualization test
ros2 launch humanoid_arm_description display.launch.py
```

You should see RViz open with the robot model and joint sliders.

## Quick Setup Script

For automation, use the provided setup script:

```bash
./setup_workspace.sh
```

## Troubleshooting

### "rosdep: command not found"
```bash
sudo apt install python3-rosdep
```

### "No module named 'colcon'"
```bash
sudo apt install python3-colcon-common-extensions
```

### Gazebo packages not found
```bash
# Check if Gazebo Harmonic is installed
gz sim --version

# If not, install:
sudo apt install gz-harmonic
```

### Permission denied for joystick
```bash
sudo chmod 666 /dev/input/js0
# Or add user to input group:
sudo usermod -a -G input $USER
```

## Next Steps

After installation:
1. Read [CLAUDE.md](CLAUDE.md) for complete usage guide
2. Test mock hardware: `ros2 launch humanoid_arm_bringup robot.launch.py`
3. Test Gazebo simulation: `ros2 launch humanoid_arm_bringup full_system.launch.py`
4. Review PID tuning guide: [PID_README.md](PID_README.md)

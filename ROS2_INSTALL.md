# ROS2 Jazzy Installation (Binary Package)

Complete guide for installing ROS2 Jazzy from binary packages on Ubuntu 24.04.

## Prerequisites

### System Requirements
- **OS**: Ubuntu 24.04 (Noble Numbat) - **Required for ROS2 Jazzy**
- **RAM**: 4GB minimum, 8GB+ recommended
- **Storage**: 5GB free space
- **Internet connection**: Required for package download

### Update System
```bash
sudo apt update && sudo apt upgrade -y
```

---

## 1. Set Locale

Ensure locale supports UTF-8:

```bash
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Verify
locale
```

---

## 2. Add ROS2 APT Repository

### Enable Ubuntu Universe Repository

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
```

### Add ROS2 GPG Key

```bash
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Add Repository to Sources List

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Update Package List

```bash
sudo apt update
```

---

## 3. Install ROS2 Jazzy

### Option 1: Desktop Full (Recommended)

Includes ROS2 core, RViz, demos, and tutorials:

```bash
sudo apt install ros-jazzy-desktop-full
```

**Packages included:**
- `rclcpp`, `rclpy` - C++ and Python client libraries
- `rviz2` - 3D visualization
- `robot_state_publisher` - Publishes robot state to TF
- `tf2` - Transform library
- `ros2cli` - Command-line tools
- Demo nodes and tutorials

**Disk space:** ~2.5GB

### Option 2: Desktop (Without Demos)

Includes ROS2 core and RViz, but no demos:

```bash
sudo apt install ros-jazzy-desktop
```

**Disk space:** ~2GB

### Option 3: ROS-Base (Minimal)

Core ROS2 only (no GUI tools):

```bash
sudo apt install ros-jazzy-ros-base
```

**Disk space:** ~500MB

**Use case:** Embedded systems, servers, Docker containers

---

## 4. Install Development Tools

Required for building ROS2 packages:

```bash
sudo apt install ros-dev-tools
```

**Includes:**
- `python3-colcon-common-extensions` - Build system
- `python3-rosdep` - Dependency management
- `python3-vcstool` - Version control tools
- `python3-flake8` - Python linting
- Testing tools

---

## 5. Initialize rosdep

rosdep manages ROS package dependencies:

```bash
# Initialize (first time only)
sudo rosdep init

# Update package lists
rosdep update
```

---

## 6. Source ROS2 Environment

### Temporary (Current Terminal)

```bash
source /opt/ros/jazzy/setup.bash
```

### Permanent (Add to ~/.bashrc)

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Important:** This must be sourced in every terminal where you use ROS2 commands.

---

## 7. Verify Installation

### Check ROS2 Version

```bash
ros2 --version
```

**Expected output:**
```
ros2 cli version jazzy
```

### Check Environment Variables

```bash
printenv | grep -i ROS
```

**Expected output:**
```
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=jazzy
```

### Test Basic Commands

```bash
# List installed packages
ros2 pkg list

# Show executables in a package
ros2 pkg executables demo_nodes_cpp

# Check available topics
ros2 topic list
```

### Test Demo Nodes

```bash
# Terminal 1: Start talker
ros2 run demo_nodes_cpp talker

# Terminal 2: Start listener
ros2 run demo_nodes_cpp listener
```

**Expected:** Listener receives messages from talker

---

## 8. Install Additional ROS2 Packages

The following packages are not included in desktop-full but are useful for development:

### Build and Testing Tools

```bash
sudo apt install \
  python3-colcon-mixin \
  python3-pytest-cov \
  python3-pytest-repeat \
  python3-pytest-rerunfailures
```

### Documentation Tools

```bash
sudo apt install \
  ros-jazzy-ament-cmake-doxygen \
  doxygen \
  graphviz
```

### Debugging Tools

```bash
sudo apt install \
  ros-jazzy-launch-testing \
  ros-jazzy-launch-testing-ament-cmake
```

---

## 9. Configure Colcon

### Add Colcon Mixins

Mixins provide convenient build configurations:

```bash
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```

### Enable Colcon Tab Completion

```bash
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 10. Create ROS2 Workspace

### Create Workspace Directory

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### Test Workspace Build

```bash
# Build empty workspace
colcon build

# Source workspace
source install/setup.bash
```

### Add Workspace to ~/.bashrc

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

**Important:** Workspace setup.bash must be sourced AFTER ROS2 setup.bash

**Order in ~/.bashrc:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

## 11. Install Core ROS2 Packages for This Project

### URDF and Robot Description

```bash
sudo apt install \
  ros-jazzy-xacro \
  ros-jazzy-urdf \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui
```

### Visualization

```bash
sudo apt install \
  ros-jazzy-rviz2 \
  ros-jazzy-rviz-common \
  ros-jazzy-rviz-default-plugins
```

### Transform Library

```bash
sudo apt install \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-tf2-geometry-msgs
```

---

## 12. Uninstalling ROS2

If you need to remove ROS2:

```bash
# Remove ROS2 packages
sudo apt remove ~nros-jazzy-* && sudo apt autoremove

# Remove repository
sudo rm /etc/apt/sources.list.d/ros2.list

# Remove GPG key
sudo rm /usr/share/keyrings/ros-archive-keyring.gpg

# Remove from bashrc
sed -i '/ros\/jazzy/d' ~/.bashrc
sed -i '/ros2_ws/d' ~/.bashrc
```

---

## 13. Upgrading ROS2 Packages

To update to latest package versions:

```bash
sudo apt update
sudo apt upgrade
```

ROS2 Jazzy will receive updates until May 2029 (5-year LTS support).

---

## Troubleshooting

### Issue: "Unable to locate package ros-jazzy-desktop"

**Solution:**
```bash
# Verify Ubuntu version (must be 24.04)
lsb_release -a

# Check repository is added
cat /etc/apt/sources.list.d/ros2.list

# Update package list
sudo apt update
```

### Issue: "GPG error" when running apt update

**Solution:**
```bash
# Re-add GPG key
sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update
```

### Issue: "command 'ros2' not found"

**Solution:**
```bash
# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Verify installation
ls /opt/ros/jazzy

# Add to bashrc if not already added
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### Issue: rosdep init fails with "already initialized"

**Solution:**
```bash
# Remove existing rosdep
sudo rm -rf /etc/ros/rosdep

# Reinitialize
sudo rosdep init
rosdep update
```

### Issue: Packages not found by rosdep

**Solution:**
```bash
# Update rosdep
rosdep update

# Check rosdep database
rosdep db

# Install with -r flag to skip missing packages
rosdep install --from-paths src --ignore-src -y -r
```

### Issue: Workspace overlay not working

**Solution:**
```bash
# Check source order in ~/.bashrc
cat ~/.bashrc | grep source

# ROS2 MUST be sourced before workspace
# Correct order:
# source /opt/ros/jazzy/setup.bash
# source ~/ros2_ws/install/setup.bash

# Re-source
source ~/.bashrc
```

---

## Performance Tips

### Use Colcon Mixins

```bash
# Build with release optimization
colcon build --mixin release

# Build with debug symbols
colcon build --mixin debug

# List available mixins
colcon mixin list
```

### Parallel Building

```bash
# Use all CPU cores (default)
colcon build --parallel-workers $(nproc)

# Limit parallel jobs (lower memory usage)
colcon build --parallel-workers 2
```

### Symlink Install (Faster Iteration)

```bash
# Create symlinks instead of copying files
colcon build --symlink-install

# Changes to Python files don't require rebuild
```

---

## Best Practices

### 1. Always Use Workspaces

Don't install packages system-wide. Use workspaces for development:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Clone packages here
cd ~/ros2_ws
colcon build
```

### 2. Source Order Matters

Overlay order determines which packages take precedence:
```bash
source /opt/ros/jazzy/setup.bash               # Base ROS2
source ~/ros2_control_ws/install/setup.bash    # ros2_control overlay
source ~/moveit2_ws/install/setup.bash         # MoveIt2 overlay
source ~/ros2_ws/install/setup.bash            # Your project (highest priority)
```

### 3. Clean Builds

If you encounter strange build issues:
```bash
cd ~/ros2_ws
rm -rf build install log
colcon build
```

### 4. Use rosdep for Dependencies

Always use rosdep to install package dependencies:
```bash
rosdep install --from-paths src --ignore-src -y -r
```

---

## Environment Variables Reference

After sourcing ROS2, these variables are set:

```bash
ROS_VERSION=2                          # ROS version (2 = ROS2)
ROS_PYTHON_VERSION=3                   # Python version
ROS_DISTRO=jazzy                       # Distribution name
AMENT_PREFIX_PATH=/opt/ros/jazzy       # Ament package prefix
CMAKE_PREFIX_PATH=/opt/ros/jazzy       # CMake search path
PYTHONPATH=/opt/ros/jazzy/...          # Python module path
LD_LIBRARY_PATH=/opt/ros/jazzy/lib     # Shared library path
PATH=/opt/ros/jazzy/bin:...            # Executable search path
ROS_LOCALHOST_ONLY=0                   # Network configuration
```

---

## Next Steps

After installing ROS2 Jazzy:

1. **Install ros2_control from source** - See [ROS2_CONTROL_INSTALL.md](ROS2_CONTROL_INSTALL.md)
2. **Install MoveIt2 from source** - See [MOVEIT2_INSTALL.md](MOVEIT2_INSTALL.md)
3. **Install additional tools** - See [ADDITIONAL_TOOLS_INSTALL.md](ADDITIONAL_TOOLS_INSTALL.md)
4. **Clone and build humanoid arm project** - See [README.md](README.md)

---

## References

- ROS2 Jazzy Documentation: https://docs.ros.org/en/jazzy/
- Installation Guide: https://docs.ros.org/en/jazzy/Installation.html
- Tutorials: https://docs.ros.org/en/jazzy/Tutorials.html
- Jazzy Release Page: https://www.ros.org/reps/rep-2000.html#jazzy-jalisco-may-2024-may-2029
- Ubuntu 24.04 Packages: https://packages.ubuntu.com/noble/

---

## Support

ROS2 Jazzy Support Timeline:
- **Released:** May 2024
- **EOL:** May 2029 (5-year LTS)
- **Ubuntu:** 24.04 (Noble Numbat) only

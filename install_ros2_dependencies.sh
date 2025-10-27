#!/bin/bash
# Complete ROS2 Jazzy and Dependencies Installation Script
# For Ubuntu 24.04 (Noble)
# Run with: sudo ./install_ros2_dependencies.sh

set -e  # Exit on error

echo "========================================="
echo "ROS2 Jazzy Installation Script"
echo "Humanoid Arm 5-DOF Project"
echo "========================================="
echo ""

# Check Ubuntu version
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_CODENAME" != "noble" ]; then
        echo "WARNING: This script is designed for Ubuntu 24.04 (Noble)."
        echo "Your version: $VERSION_CODENAME"
        read -p "Continue anyway? (y/N) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
fi

# Check if running with sudo
if [ "$EUID" -ne 0 ]; then
    echo "ERROR: Please run with sudo"
    echo "Usage: sudo ./install_ros2_dependencies.sh"
    exit 1
fi

echo "Step 1: Setting up system prerequisites..."
apt update
apt install -y software-properties-common curl gnupg lsb-release

echo ""
echo "Step 2: Adding ROS2 apt repository..."
# Install curl if not present
apt install -y curl

# Add ROS2 GPG key
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Add universe repository
add-apt-repository universe -y

echo ""
echo "Step 3: Updating package lists..."
apt update

echo ""
echo "Step 4: Installing ROS2 Jazzy Desktop Full..."
apt install -y ros-jazzy-desktop-full

echo ""
echo "Step 5: Installing development tools..."
apt install -y \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    git \
    wget

echo ""
echo "Step 6: Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
    echo "rosdep initialized"
else
    echo "rosdep already initialized, skipping..."
fi

# Run rosdep update as the actual user (not root)
ACTUAL_USER=${SUDO_USER:-$USER}
echo "Updating rosdep as user: $ACTUAL_USER"
sudo -u $ACTUAL_USER rosdep update

echo ""
echo "Step 7: Installing ros2_control packages..."
apt install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-control-msgs \
    ros-jazzy-control-toolbox \
    ros-jazzy-hardware-interface \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-effort-controllers \
    ros-jazzy-position-controllers \
    ros-jazzy-velocity-controllers

echo ""
echo "Step 8: Installing Gazebo Harmonic and ROS2 integration..."
apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-gz-ros2-control

# Install Gazebo Harmonic if not present
if ! command -v gz &> /dev/null; then
    echo "Installing Gazebo Harmonic..."
    wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    apt update
    apt install -y gz-harmonic
fi

echo ""
echo "Step 9: Installing MoveIt2..."
apt install -y \
    ros-jazzy-moveit \
    ros-jazzy-moveit-ros-planning \
    ros-jazzy-moveit-ros-planning-interface \
    ros-jazzy-moveit-ros-perception \
    ros-jazzy-moveit-servo \
    ros-jazzy-moveit-visual-tools \
    ros-jazzy-moveit-setup-assistant

echo ""
echo "Step 10: Installing additional utilities..."
apt install -y \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-tf2-tools \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-plotjuggler-ros

echo ""
echo "Step 11: Installing Python dependencies..."
sudo -u $ACTUAL_USER pip3 install --user \
    numpy \
    matplotlib \
    scipy

echo ""
echo "Step 12: Setting up environment..."
# Add ROS2 sourcing to bashrc if not already present
BASHRC_FILE="/home/$ACTUAL_USER/.bashrc"
if ! grep -q "source /opt/ros/jazzy/setup.bash" "$BASHRC_FILE"; then
    echo "" >> "$BASHRC_FILE"
    echo "# ROS2 Jazzy setup" >> "$BASHRC_FILE"
    echo "source /opt/ros/jazzy/setup.bash" >> "$BASHRC_FILE"
    echo "Added ROS2 sourcing to $BASHRC_FILE"
fi

echo ""
echo "Step 13: Setting up colcon mixins..."
sudo -u $ACTUAL_USER colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml 2>/dev/null || true
sudo -u $ACTUAL_USER colcon mixin update 2>/dev/null || true

echo ""
echo "========================================="
echo "Installation Complete!"
echo "========================================="
echo ""
echo "Installed components:"
echo "  ✓ ROS2 Jazzy Desktop Full"
echo "  ✓ Development tools (colcon, rosdep, vcstool)"
echo "  ✓ ros2_control and controllers"
echo "  ✓ Gazebo Harmonic with ROS2 integration"
echo "  ✓ MoveIt2 motion planning"
echo "  ✓ Utilities (xacro, joint_state_publisher, etc.)"
echo "  ✓ Python dependencies"
echo ""
echo "Next steps:"
echo "  1. Close and reopen your terminal (or run: source ~/.bashrc)"
echo "  2. Verify installation: ros2 --version"
echo "  3. Build workspace: ./setup_workspace.sh"
echo ""
echo "For joystick support, add your user to input group:"
echo "  sudo usermod -a -G input $ACTUAL_USER"
echo "  (logout/login required after this)"
echo ""
